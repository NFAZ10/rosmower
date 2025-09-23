#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def wrap(a):
    while a >  math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

class PID:
    def __init__(self, kp, ki, kd, i_lim=0.4, out_lim=0.6):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i, self.pe = 0.0, None
        self.i_lim, self.out_lim = i_lim, out_lim

    def reset(self): self.i, self.pe = 0.0, None

    def step(self, e, dt):
        if dt <= 0.0: return 0.0
        self.i += e * dt
        self.i = max(-self.i_lim, min(self.i, self.i_lim))
        d = 0.0 if self.pe is None else (e - self.pe) / dt
        self.pe = e
        u = self.kp*e + self.ki*self.i + self.kd*d
        return max(-self.out_lim, min(u, self.out_lim))

class StabilityController(Node):
    def __init__(self):
        super().__init__('stability_controller')
        # ---- params (tune) ----
        self.declare_parameter('cmd_in', '/cmd_vel_raw')
        self.declare_parameter('cmd_out', '/cmd_vel')
        self.declare_parameter('use_ekf_yaw', True)
        self.declare_parameter('max_lin_accel', 0.8)         # m/s^2
        self.declare_parameter('yaw_kp', 1.4)
        self.declare_parameter('yaw_ki', 0.0)
        self.declare_parameter('yaw_kd', 0.12)
        self.declare_parameter('max_ang_correction', 0.5)    # rad/s limit
        self.declare_parameter('capture_lin_min', 0.15)      # only hold heading if moving > this
        self.declare_parameter('deadband_ang_cmd', 0.01)     # treat as "no user turn"
        self.declare_parameter('rate', 30.0)

        self.pid = PID(
            self.get_parameter('yaw_kp').value,
            self.get_parameter('yaw_ki').value,
            self.get_parameter('yaw_kd').value,
            out_lim=self.get_parameter('max_ang_correction').value
        )

        self.cmd_in = Twist()
        self.cmd_s = Twist()
        self.yaw = None
        self.yaw_sp = None
        self.last = time.time()

        # subs/pubs
        if self.get_parameter('use_ekf_yaw').value:
            self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.cb_odom, 30)
        else:
            from sensor_msgs.msg import Imu
            self.sub_imu = self.create_subscription(Imu, '/mavros/imu/data', self.cb_imu, 50)

        self.sub_cmd = self.create_subscription(Twist, self.get_parameter('cmd_in').value, self.cb_cmd, 10)
        self.pub_cmd = self.create_publisher(Twist, self.get_parameter('cmd_out').value, 10)
        self.timer = self.create_timer(1.0/self.get_parameter('rate').value, self.tick)

        self.get_logger().info('StabilityController running')

    def cb_cmd(self, msg: Twist):
        self.cmd_in = msg
        if abs(msg.angular.z) > self.get_parameter('deadband_ang_cmd').value:
            # user (or planner) is turning → no hold
            self.yaw_sp = None
            self.pid.reset()

    def cb_imu(self, imu):
        q = imu.orientation
        syc = 2.0*(q.w*q.z + q.x*q.y)
        cyc = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(syc, cyc)

    def cb_odom(self, odom):
        q = odom.pose.pose.orientation
        syc = 2.0*(q.w*q.z + q.x*q.y)
        cyc = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(syc, cyc)

    def tick(self):
        now = time.time()
        dt = now - self.last
        self.last = now
        if dt <= 0: return

        # accel limit on linear.x
        amax = self.get_parameter('max_lin_accel').value
        dv   = self.cmd_in.linear.x - self.cmd_s.linear.x
        max_dv = amax * dt
        if abs(dv) > max_dv: dv = math.copysign(max_dv, dv)
        self.cmd_s.linear.x += dv

        out = Twist()
        out.linear.x = self.cmd_s.linear.x
        out.angular.z = self.cmd_in.angular.z

        # capture yaw setpoint when going straight & moving
        if (abs(self.cmd_in.angular.z) <= self.get_parameter('deadband_ang_cmd').value
            and abs(out.linear.x) >= self.get_parameter('capture_lin_min').value
            and self.yaw is not None):
            if self.yaw_sp is None:
                self.yaw_sp = self.yaw

        # apply correction if holding
        if self.yaw_sp is not None and self.yaw is not None:
            e = wrap(self.yaw_sp - self.yaw)
            out.angular.z += self.pid.step(e, dt)

        self.pub_cmd.publish(out)

def main():
    rclpy.init()
    rclpy.spin(StabilityController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
