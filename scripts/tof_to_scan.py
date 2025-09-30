#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math

# angles in degrees: left=+90, front=0, right=-90
ANGLES_DEG = [0, 50, 60]   # [ch0, ch1, ch2] but swapped to match
# Actually: user said ch0=left(+90), ch1=right(-90), ch2=front(0)
# So angles should be [90, -90, 0]
ANGLES_RAD = [math.radians(a) for a in ANGLES_DEG]

class ToFScan(Node):
    def __init__(self):
        super().__init__('tof_to_scan')
        self.sub = self.create_subscription(String, 'vl53_distances', self.cb, 10)
        self.pub = self.create_publisher(LaserScan, 'scan', 10)

        self.get_logger().info("ToF -> LaserScan bridge started")

    def cb(self, msg: String):
        # expected: "ch0=123,ch1=456,ch2=789"
        parts = msg.data.split(',')
        if len(parts) < 3:
            return

        # parse each channel to meters
        distances = []
        for p in parts[:3]:
            val = p.split('=')[1] if '=' in p else ""
            if val.isdigit():
                distances.append(float(val)/1000.0)
            else:
                distances.append(float('inf'))

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "base_link"
        scan.angle_min = min(ANGLES_RAD)
        scan.angle_max = max(ANGLES_RAD)
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (len(distances)-1)
        scan.range_min = 0.02
        scan.range_max = 2.0  # adjust for VL53L0X range
        # Place distances in correct order: left, right, front
        # If you want exactly in left->front->right order adjust angles accordingly
        scan.ranges = distances

        self.pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = ToFScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
