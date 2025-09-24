#!/usr/bin/env python3
import threading, time, serial, math
from math import copysign
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, String
from std_srvs.srv import Trigger, SetBool

# NEW:
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
try:
    from tf_transformations import quaternion_from_euler  # if available
except Exception:
    quaternion_from_euler = None
from tf2_ros import TransformBroadcaster

def clamp(n, lo, hi):
    return lo if n < lo else hi if n > hi else n

class HoverboardBridge(Node):
    def __init__(self):
        super().__init__('hoverboard_bridge')

        # ---------- Parameters ----------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wait_after_open', 2.0)
        self.declare_parameter('reset_dtr', True)
        self.declare_parameter('read_timeout', 0.05)
        self.declare_parameter('verbose_serial', True)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('max_lin', 1.0)      # m/s for full-scale PWM (only used to compute PWM)
        self.declare_parameter('max_ang', 2.0)      # rad/s for full-scale PWM (only used to compute PWM)
        self.declare_parameter('stat_period', 0.5)
        self.declare_parameter('arm_on_start', True)

        # NEW: kinematics + publishing
        self.declare_parameter('wheel_radius', 0.16)       # meters (set yours)
        self.declare_parameter('wheel_separation', 0.52)   # meters (set yours)
        self.declare_parameter('ticks_per_rev', 0)         # >0 enables encoder mode if ENC lines are parsed
        self.declare_parameter('joint_state_rate', 50.0)   # Hz
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_odom_tf', True)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('left_joint_name', 'left_wheel_joint')
        self.declare_parameter('right_joint_name', 'right_wheel_joint')

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = int(self.get_parameter('baud').get_parameter_value().integer_value)
        wait_after_open = float(self.get_parameter('wait_after_open').get_parameter_value().double_value)
        reset_dtr = bool(self.get_parameter('reset_dtr').get_parameter_value().bool_value)
        read_timeout = float(self.get_parameter('read_timeout').get_parameter_value().double_value)
        verbose_serial = bool(self.get_parameter('verbose_serial').get_parameter_value().bool_value)
        self._verbose_serial = verbose_serial

        # NEW: cache params
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_sep    = float(self.get_parameter('wheel_separation').value)
        self.ticks_per_rev = int(self.get_parameter('ticks_per_rev').value)
        self.js_rate      = float(self.get_parameter('joint_state_rate').value)
        self.publish_odom = bool(self.get_parameter('publish_odom').value)
        self.publish_odom_tf = bool(self.get_parameter('publish_odom_tf').value)
        self.odom_frame   = self.get_parameter('odom_frame_id').value
        self.base_frame   = self.get_parameter('base_frame_id').value
        self.left_joint   = self.get_parameter('left_joint_name').value
        self.right_joint  = self.get_parameter('right_joint_name').value

        # ---------- Serial ----------
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=read_timeout, write_timeout=0.15)
            try:
                if reset_dtr:
                    self.ser.dtr = False; time.sleep(0.05); self.ser.dtr = True
                    self.get_logger().info('Toggled DTR on serial port')
            except Exception:
                pass
            self.get_logger().info(f'Opened serial: {port} @ {baud} (timeout={read_timeout})')
            if wait_after_open and wait_after_open > 0:
                self.get_logger().info(f'Waiting {wait_after_open}s after opening serial for board startup')
                time.sleep(wait_after_open)
        except Exception as e:
            self.get_logger().fatal(f'Failed to open serial {port}: {e}')
            raise

        # ---------- Pub/Sub ----------
        # Keep your topics/services
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Int16, 'blade_pwm', self.on_blade_pwm, 10)
        self.state_pub = self.create_publisher(String, 'driver_state', 10)

        # NEW: joint states + odom publishers (latched-ish QoS)
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.history = QoSHistoryPolicy.KEEP_LAST

        self.js_pub = self.create_publisher(JointState, 'joint_states', qos)
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos) if self.publish_odom else None
        self.tf_br = TransformBroadcaster(self) if self.publish_odom_tf else None

        # ---------- Services ----------
        self.create_service(Trigger, 'arm', self.srv_arm)
        self.create_service(Trigger, 'stop', self.srv_stop)
        self.create_service(SetBool, 'brake', self.srv_brake)
        self.create_service(SetBool, 'dirinv_right', self.srv_dirinv_right)
        self.create_service(SetBool, 'dirinv_left', self.srv_dirinv_left)

        # ---------- Timers ----------
        period = float(self.get_parameter('stat_period').get_parameter_value().double_value)
        self.timer = self.create_timer(period, self.poll_stat)

        if self.get_parameter('arm_on_start').get_parameter_value().bool_value:
            self.send_line('ARM')

        # Reader thread (non-blocking)
        self._rx_buf = bytearray()
        self._rx_running = True
        self._rx_thread = threading.Thread(target=self._reader, daemon=True)
        self._rx_thread.start()

        # NEW: state for JS/odom integration
        self.last_t = self.get_clock().now()
        self.cmd_wl = 0.0  # rad/s
        self.cmd_wr = 0.0  # rad/s
        self.pos_l = 0.0   # rad
        self.pos_r = 0.0   # rad
        self.vel_l = 0.0   # rad/s
        self.vel_r = 0.0   # rad/s

        # odom pose
        self.x = 0.0; self.y = 0.0; self.th = 0.0

        # encoder mode helpers
        self._enc_last_l = None
        self._enc_last_r = None
        self._enc_last_stamp = None
        self._enc_active = False  # set True after first good ENC parse

        # NEW: publisher timer
        self.create_timer(1.0 / self.js_rate, self._publish_joint_states)

    # ---------------- Serial helpers ----------------
    def send_line(self, line: str):
        try:
            msg = (line + '\r\n').encode('ascii')
            self.ser.write(msg)
            self.ser.flush()
            if getattr(self, '_verbose_serial', False):
                self.get_logger().info(f"TX: {line}")
        except Exception as e:
            self.get_logger().error(f'write failed: {e}')

    def _reader(self):
        while self._rx_running:
            try:
                data = self.ser.read(256)
                if data:
                    self._rx_buf.extend(data)
                    while b'\n' in self._rx_buf:
                        line, _, rest = self._rx_buf.partition(b'\n')
                        self._rx_buf = bytearray(rest)
                        text = line.decode('ascii', errors='ignore').strip()
                        if text:
                            if getattr(self, '_verbose_serial', False):
                                self.get_logger().info(f"RX: {text}")
                            self.state_pub.publish(String(data=text))
                            # NEW: try to parse encoder lines
                            self._maybe_parse_encoders(text)
            except Exception as e:
                try:
                    self.get_logger().warning(f'serial read err: {e}')
                except Exception:
                    self.get_logger().info(f'serial read err: {e}')
                time.sleep(0.05)

    # ---------------- ROS callbacks ----------------
    def on_cmd_vel(self, msg: Twist):
        # Maintain your PWM output protocol
        max_pwm = int(self.get_parameter('max_pwm').get_parameter_value().integer_value)
        max_lin = float(self.get_parameter('max_lin').get_parameter_value().double_value)
        max_ang = float(self.get_parameter('max_ang').get_parameter_value().double_value)

        l_pwm = clamp(int((msg.linear.x / max_lin) * max_pwm - (msg.angular.z / max_ang) * max_pwm), -max_pwm, max_pwm)
        r_pwm = clamp(int((msg.linear.x / max_lin) * max_pwm + (msg.angular.z / max_ang) * max_pwm), -max_pwm, max_pwm)
        self.send_line(f'VEL {l_pwm} {r_pwm}')

        # NEW: also compute desired wheel angular velocities (rad/s) for fake JS/odom
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        wl = (v - 0.5 * w * self.wheel_sep) / self.wheel_radius
        wr = (v + 0.5 * w * self.wheel_sep) / self.wheel_radius
        self.cmd_wl = wl
        self.cmd_wr = wr

    def on_blade_pwm(self, msg: Int16):
        val = clamp(int(msg.data), -255, 255)
        self.send_line(f'BLADE {val}')

    def poll_stat(self):
        self.send_line('STAT')

    # ---------------- Services ----------------
    def srv_arm(self, req, resp):
        self.send_line('ARM'); resp.success = True; resp.message = 'ARM sent'; return resp

    def srv_stop(self, req, resp):
        self.send_line('STOP'); resp.success = True; resp.message = 'STOP sent'; return resp

    def srv_brake(self, req: SetBool.Request, resp: SetBool.Response):
        self.send_line(f'BRAKE ALL {1 if req.data else 0}')
        resp.success = True; resp.message = f'BRAKE set to {"ENGAGED" if req.data else "RELEASED"}'; return resp

    def srv_dirinv_right(self, req: SetBool.Request, resp: SetBool.Response):
        self.send_line(f'DIRINV R {1 if req.data else 0}')
        resp.success = True; resp.message = f'DIRINV R set to {req.data}'; return resp

    def srv_dirinv_left(self, req: SetBool.Request, resp: SetBool.Response):
        self.send_line(f'DIRINV L {1 if req.data else 0}')
        resp.success = True; resp.message = f'DIRINV L set to {req.data}'; return resp

    # ---------------- Encoders (optional, if MCU sends them) ----------------
    def _maybe_parse_encoders(self, text: str):
        """
        Expected examples (you pick one on the Arduino and keep it consistent):
          "ENC 12345 12560"                       # ticks L R (cumulative)
          "ENC,L:12345,R:12560"                  # labeled
          "ENC 12345 12560 DT:20"                # optional dt in ms
        Set param ticks_per_rev > 0 to enable.
        """
        if self.ticks_per_rev <= 0:
            return
        if not text.startswith('ENC'):
            return

        # crude parse for ints in the line
        parts = text.replace(',', ' ').replace(':', ' ').split()
        vals = [int(p) for p in parts if p.lstrip('-').isdigit()]
        if len(vals) < 2:
            return
        ticks_l, ticks_r = vals[0], vals[1]
        now = self.get_clock().now()

        if self._enc_last_l is None:
            self._enc_last_l, self._enc_last_r = ticks_l, ticks_r
            self._enc_last_stamp = now
            self._enc_active = True
            return

        # deltas
        dt = (now - self._enc_last_stamp).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        dL = ticks_l - self._enc_last_l
        dR = ticks_r - self._enc_last_r

        rad_per_tick = (2.0 * math.pi) / float(self.ticks_per_rev)
        dth_l = dL * rad_per_tick
        dth_r = dR * rad_per_tick

        self.pos_l += dth_l
        self.pos_r += dth_r
        self.vel_l = dth_l / dt
        self.vel_r = dth_r / dt

        # also update base pose from encoders
        v_l = self.vel_l * self.wheel_radius
        v_r = self.vel_r * self.wheel_radius
        v = 0.5 * (v_r + v_l)
        w = (v_r - v_l) / self.wheel_sep
        self._integrate_odom(v, w, dt)

        self._enc_last_l, self._enc_last_r = ticks_l, ticks_r
        self._enc_last_stamp = now

    # ---------------- JointStates / Odom publishing ----------------
    def _publish_joint_states(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if dt <= 0.0:
            return

        if not self._enc_active:
            # Fallback: integrate commanded wheel speeds
            self.vel_l = self.cmd_wl
            self.vel_r = self.cmd_wr
            self.pos_l += self.vel_l * dt
            self.pos_r += self.vel_r * dt

            if self.publish_odom:
                v_l = self.vel_l * self.wheel_radius
                v_r = self.vel_r * self.wheel_radius
                v = 0.5 * (v_r + v_l)
                w = (v_r - v_l) / self.wheel_sep
                self._integrate_odom(v, w, dt)

        # JointState
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [self.left_joint, self.right_joint]
        js.position = [self.pos_l, self.pos_r]
        js.velocity = [self.vel_l, self.vel_r]
        self.js_pub.publish(js)

        # Odom + TF (optional)
        if self.publish_odom and (self.odom_pub is not None):
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            q = self._yaw_to_quat(self.th)
            odom.pose.pose.orientation = q
            # simple covariance defaults
            odom.twist.twist.linear.x  = self.v_last if hasattr(self, 'v_last') else 0.0
            odom.twist.twist.angular.z = self.w_last if hasattr(self, 'w_last') else 0.0
            self.odom_pub.publish(odom)

        if self.publish_odom_tf and (self.tf_br is not None):
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            q = self._yaw_to_quat(self.th)
            t.transform.rotation = q
            self.tf_br.sendTransform(t)

    def _integrate_odom(self, v, w, dt):
        # Save last for odom twist
        self.v_last = v
        self.w_last = w
        if abs(w) < 1e-6:
            self.x += v * math.cos(self.th) * dt
            self.y += v * math.sin(self.th) * dt
        else:
            self.x += (v / w) * (math.sin(self.th + w * dt) - math.sin(self.th))
            self.y += (v / w) * (-math.cos(self.th + w * dt) + math.cos(self.th))
            self.th += w * dt

    def _yaw_to_quat(self, yaw):
        if quaternion_from_euler is not None:
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        else:
            # minimal dependency-free quaternion from yaw
            half = 0.5 * yaw
            qx = 0.0
            qy = 0.0
            qz = math.sin(half)
            qw = math.cos(half)
        from geometry_msgs.msg import Quaternion
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    # ---------------- Shutdown ----------------
    def destroy_node(self):
        self._rx_running = False
        try:
            self._rx_thread.join(timeout=0.5)
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass
        return super().destroy_node()

def main():
    rclpy.init()
    node = HoverboardBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
