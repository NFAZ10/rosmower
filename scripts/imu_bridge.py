#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class MavrosImuBridge(Node):
    def __init__(self):
        super().__init__('mavros_imu_bridge')

        # Declare parameters for flexibility
        self.declare_parameter('mavros_imu_topic', '/mavros/imu/data')
        self.declare_parameter('output_imu_topic', '/imu/data_raw')

        self.mavros_imu_topic = self.get_parameter('mavros_imu_topic').get_parameter_value().string_value
        self.output_imu_topic = self.get_parameter('output_imu_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            Imu,
            self.mavros_imu_topic,
            self.imu_callback,
            10
        )
        self.publisher = self.create_publisher(Imu, self.output_imu_topic, 10)

        self.get_logger().info(f"🔁 Relaying IMU: {self.mavros_imu_topic} → {self.output_imu_topic}")

    def imu_callback(self, msg: Imu):
        # Optionally, modify the message (e.g., fix frame_id)
        msg.header.frame_id = 'imu_link'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MavrosImuBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
