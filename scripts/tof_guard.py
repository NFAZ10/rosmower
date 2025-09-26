import math, time
from typing import List
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from .vl53l0x_multi import VL53Group

class ToFGuard(Node):
    def __init__(self):
        super().__init__('tof_guard')

        # GPIOs (BCM) for XSHUT and target I2C addrs
        self.declare_parameter('xshut_pins', [27, 22, 23, 24, 25])
        self.declare_parameter('addresses',  [0x30, 0x31, 0x32, 0x33, 0x34])

        # Frames / topics for 5 sensors (no blade)
        self.declare_parameter('frames',  ['tof_front_left','tof_front_right','tof_left','tof_right','tof_rear'])
        self.declare_parameter('topics',  ['tof/front_left/range','tof/front_right/range','tof/left/range','tof/right/range','tof/rear/range'])

        # Per-sensor stop thresholds (cm): fronts strictest
        self.declare_parameter('thresholds_cm', [35, 35, 25, 25, 30])

        # Misc
        self.declare_parameter('poll_hz', 20.0)
        self.declare_parameter('cmd_vel_in',  '/cmd_vel_in')
        self.declare_parameter('cmd_vel_out', '/cmd_vel')
        self.declare_parameter('cooldown_s', 0.4)
        self.declare_parameter('field_of_view_deg', 25.0)
        self.declare_parameter('max_range_m', 2.0)
        self.declare_parameter('min_range_m', 0.03)

        xshut_pins = list(self.get_parameter('xshut_pins').value)
        addresses  = list(self.get_parameter('addresses').value)
        self.frames = list(self.get_parameter('frames').value)
        topics     = list(self.get_parameter('topics').value)
        thresholds_cm = list(self.get_parameter('thresholds_cm').value)

        assert len(xshut_pins) == len(addresses) == len(self.frames) == len(topics) == len(thresholds_cm)

        self.poll_hz = float(self.get_parameter('poll_hz').value)
        self.cooldown_s = float(self.get_parameter('cooldown_s').value)
        self.cmd_in_name  = self.get_parameter('cmd_vel_in').value
        self.cmd_out_name = self.get_parameter('cmd_vel_out').value
        self.fov = math.radians(float(self.get_parameter('field_of_view_deg').value))
        self.max_range = float(self.get_parameter('max_range_m').value)
        self.min_range = float(self.get_parameter('min_range_m').value)
        self.thresholds_m = [c/100.0 for c in thresholds_cm]

        # Bring sensors online
        self.group = VL53Group(xshut_pins, addresses)
        self.sensors = self.group.bringup(timing_budget_ms=33, accuracy_mode='BETTER')

        # Publishers for Range
        self.range_pubs = [self.create_publisher(Range, t, 10) for t in topics]

        # Cmd passthrough with guard
        self.cmd_pub = self.create_publisher(Twist, self.cmd_out_name, 10)
        self._latest_cmd = Twist()
        self.create_subscription(Twist, self.cmd_in_name, self._cmd_cb, 10)

        self.last_trigger_time = 0.0
        self.create_timer(1.0/self.poll_hz, self._tick)

        self.get_logger().info(f"VL53L0X addresses: {', '.join([hex(a) for a in addresses])}")
        self.get_logger().info(f"Range topics: {topics}")
        self.get_logger().info(f"Thresholds (m): {self.thresholds_m}")
        self.get_logger().info(f"Cmd passthrough {self.cmd_in_name} -> {self.cmd_out_name} (cooldown {self.cooldown_s}s)")

    def _cmd_cb(self, msg: Twist):
        self._latest_cmd = msg

    def _tick(self):
        dmm_list: List[int] = self.group.read_all_mm()
        now = self.get_clock().now().to_msg()
        triggered = False

        for i, dmm in enumerate(dmm_list):
            rng = Range()
            rng.header = Header(stamp=now, frame_id=self.frames[i])
            rng.radiation_type = Range.INFRARED
            rng.field_of_view = self.fov
            rng.min_range = self.min_range
            rng.max_range = self.max_range

            d_m = (dmm/1000.0) if dmm > 0 else self.max_range + 0.1
            rng.range = max(self.min_range, min(d_m, self.max_range))
            self.range_pubs[i].publish(rng)

            if dmm > 0 and d_m <= self.thresholds_m[i]:
                triggered = True

        t = time.monotonic()
        if triggered:
            self.last_trigger_time = t

        if (t - self.last_trigger_time) < self.cooldown_s:
            self.cmd_pub.publish(Twist())   # stop
        else:
            self.cmd_pub.publish(self._latest_cmd)

    def destroy_node(self):
        try: self.group.stop()
        finally: super().destroy_node()

def main():
    rclpy.init()
    node = ToFGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
