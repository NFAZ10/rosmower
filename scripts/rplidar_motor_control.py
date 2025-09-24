#!/usr/bin/env python3
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan

class RPLidarMotorControl(Node):
    def __init__(self):
        super().__init__('rplidar_motor_control')
        self.declare_parameter('topic_name', '/scan')
        self.declare_parameter('node1', 'rviz')
        self.declare_parameter('node2', 'move_base')
        self.declare_parameter('seconds_between_tries', 5)

        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.node1 = self.get_parameter('node1').get_parameter_value().string_value
        self.node2 = self.get_parameter('node2').get_parameter_value().string_value
        self.seconds_between_tries = self.get_parameter('seconds_between_tries').get_parameter_value().integer_value

        self.last_scan_time = -1.0
        self.last_call = time.time() - self.seconds_between_tries

        self.scan_sub = self.create_subscription(LaserScan, self.topic_name, self.scan_cb, 10)

        self.cli_start = self.create_client(Empty, '/start_motor')
        self.cli_stop = self.create_client(Empty, '/stop_motor')

        # Timer runs at 1 Hz
        self.timer = self.create_timer(1.0, self.timer_cb)

    def scan_cb(self, msg: LaserScan):
        self.last_scan_time = self.get_clock().now().nanoseconds / 1e9

    def is_scanning(self):
        if self.last_scan_time < 0:
            return False
        return (time.time() - self.last_scan_time) < 1.0

    def get_nodes(self):
        try:
            out = subprocess.check_output(['ros2', 'node', 'list'], text=True, timeout=2)
            nodes = [line.strip() for line in out.splitlines() if line.strip()]
            return nodes
        except Exception:
            return []

    def timer_cb(self):
        # throttle attempts
        if (time.time() - self.last_call) < self.seconds_between_tries:
            return

        nodes = self.get_nodes()
        lidar_required = any(self.node1 in n for n in nodes) or any(self.node2 in n for n in nodes)

        if lidar_required and not self.is_scanning():
            if self.cli_start.wait_for_service(timeout_sec=1.0):
                req = Empty.Request()
                fut = self.cli_start.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
                if fut.done() and not fut.exception():
                    self.get_logger().info('Called /start_motor')
                else:
                    self.get_logger().error('Failed to call /start_motor')
            else:
                self.get_logger().warn('/start_motor service not available')
            self.last_call = time.time()
        elif (not lidar_required) and self.is_scanning():
            if self.cli_stop.wait_for_service(timeout_sec=1.0):
                req = Empty.Request()
                fut = self.cli_stop.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
                if fut.done() and not fut.exception():
                    self.get_logger().info('Called /stop_motor')
                else:
                    self.get_logger().error('Failed to call /stop_motor')
            else:
                self.get_logger().warn('/stop_motor service not available')
            self.last_call = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = RPLidarMotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
