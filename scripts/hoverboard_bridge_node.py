#!/usr/bin/env python3
import threading, time, serial
from math import copysign
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, String
from std_srvs.srv import Trigger, SetBool

def clamp(n, lo, hi):
    return lo if n < lo else hi if n > hi else n

class HoverboardBridge(Node):
    def __init__(self):
        super().__init__('hoverboard_bridge')
        # ---------- Parameters ----------
        self.declare_parameter('port', '/dev/ttyUSB0')          # Windows example; Linux: '/dev/ttyACM0'
        self.declare_parameter('baud', 115200)
        # Extra serial tuning
        self.declare_parameter('wait_after_open', 2.0)   # seconds to wait after opening port (Arduino reset)
        self.declare_parameter('reset_dtr', True)       # toggle DTR on open to reset boards that need it
        self.declare_parameter('read_timeout', 0.05)    # serial read timeout
        self.declare_parameter('verbose_serial', True)  # enable verbose logging of RX/TX
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('max_lin', 1.0)          # m/s for full-scale PWM
        self.declare_parameter('max_ang', 2.0)          # rad/s for full-scale PWM
        self.declare_parameter('stat_period', 0.5)      # seconds
        self.declare_parameter('arm_on_start', True)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = int(self.get_parameter('baud').get_parameter_value().integer_value)
        wait_after_open = float(self.get_parameter('wait_after_open').get_parameter_value().double_value)
        reset_dtr = bool(self.get_parameter('reset_dtr').get_parameter_value().bool_value)
        read_timeout = float(self.get_parameter('read_timeout').get_parameter_value().double_value)
        verbose_serial = bool(self.get_parameter('verbose_serial').get_parameter_value().bool_value)
        self._verbose_serial = verbose_serial

        # ---------- Serial ----------
        try:
            # Open with chosen timeout
            self.ser = serial.Serial(port, baudrate=baud, timeout=read_timeout, write_timeout=0.15)
            # Optionally toggle DTR to force Arduino reset on open
            try:
                if reset_dtr:
                    # Lower DTR for a short time then raise it (or vice-versa) depending on board
                    self.ser.dtr = False
                    time.sleep(0.05)
                    self.ser.dtr = True
                    self.get_logger().info('Toggled DTR on serial port')
            except Exception:
                # Not all serial backends expose dtr; ignore failures
                pass

            self.get_logger().info(f'Opened serial: {port} @ {baud} (timeout={read_timeout})')
            # Give board time to reboot/respond
            if wait_after_open and wait_after_open > 0:
                self.get_logger().info(f'Waiting {wait_after_open}s after opening serial for board startup')
                time.sleep(wait_after_open)
        except Exception as e:
            self.get_logger().fatal(f'Failed to open serial {port}: {e}')
            raise

        # ---------- Pub/Sub ----------
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Int16, 'blade_pwm', self.on_blade_pwm, 10)
        self.state_pub = self.create_publisher(String, 'driver_state', 10)

        # ---------- Services ----------
        self.create_service(Trigger, 'arm', self.srv_arm)
        self.create_service(Trigger, 'stop', self.srv_stop)
        self.create_service(SetBool, 'brake', self.srv_brake)              # True=engage, False=release
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

    # ---------------- Serial helpers ----------------
    def send_line(self, line: str):
        try:
            msg = (line + '\r\n').encode('ascii')
            self.ser.write(msg)
            self.ser.flush()
            if getattr(self, '_verbose_serial', False):
                # Avoid heavy logging if disabled
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
            except Exception as e:
                # warn is deprecated in some logging versions; use warning/info appropriately
                try:
                    self.get_logger().warning(f'serial read err: {e}')
                except Exception:
                    self.get_logger().info(f'serial read err: {e}')
                time.sleep(0.05)

    # ---------------- ROS callbacks ----------------
    def on_cmd_vel(self, msg: Twist):
        max_pwm = int(self.get_parameter('max_pwm').get_parameter_value().integer_value)
        max_lin = float(self.get_parameter('max_lin').get_parameter_value().double_value)
        max_ang = float(self.get_parameter('max_ang').get_parameter_value().double_value)
        # Simple differential mix
        l = clamp(int((msg.linear.x / max_lin) * max_pwm - (msg.angular.z / max_ang) * max_pwm), -max_pwm, max_pwm)
        r = clamp(int((msg.linear.x / max_lin) * max_pwm + (msg.angular.z / max_ang) * max_pwm), -max_pwm, max_pwm)
        self.send_line(f'VEL {l} {r}')

    def on_blade_pwm(self, msg: Int16):
        val = clamp(int(msg.data), -255, 255)
        self.send_line(f'BLADE {val}')

    def poll_stat(self):
        self.send_line('STAT')

    # ---------------- Services ----------------
    def srv_arm(self, req, resp):
        self.send_line('ARM')
        resp.success = True
        resp.message = 'ARM sent'
        return resp

    def srv_stop(self, req, resp):
        self.send_line('STOP')
        resp.success = True
        resp.message = 'STOP sent'
        return resp

    def srv_brake(self, req: SetBool.Request, resp: SetBool.Response):
        self.send_line(f'BRAKE ALL {1 if req.data else 0}')
        resp.success = True
        resp.message = f'BRAKE set to {"ENGAGED" if req.data else "RELEASED"}'
        return resp

    def srv_dirinv_right(self, req: SetBool.Request, resp: SetBool.Response):
        self.send_line(f'DIRINV R {1 if req.data else 0}')
        resp.success = True
        resp.message = f'DIRINV R set to {req.data}'
        return resp

    def srv_dirinv_left(self, req: SetBool.Request, resp: SetBool.Response):
        self.send_line(f'DIRINV L {1 if req.data else 0}')
        resp.success = True
        resp.message = f'DIRINV L set to {req.data}'
        return resp

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