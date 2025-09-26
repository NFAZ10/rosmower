#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32
from rclpy.qos import qos_profile_sensor_data

def valid(x: float) -> bool:
    return x is not None and not math.isnan(x) and not math.isinf(x)

class BatterySplitter(Node):
    def __init__(self):
        super().__init__('battery_splitter')

        # Parameters
        self.declare_parameter('source_topic', '/mavros/battery')
        self.declare_parameter('voltage_topic', '/voltage')
        self.declare_parameter('percent_topic', '/percent')
        self.declare_parameter('current_topic', '/current')
        self.declare_parameter('percent_scale_0_100', True)  # publish percent in 0..100

        src = self.get_parameter('source_topic').get_parameter_value().string_value
        topic_v = self.get_parameter('voltage_topic').get_parameter_value().string_value
        topic_p = self.get_parameter('percent_topic').get_parameter_value().string_value
        topic_c = self.get_parameter('current_topic').get_parameter_value().string_value
        self.scale_pct = self.get_parameter('percent_scale_0_100').get_parameter_value().bool_value

        # QoS similar to sensor data, reliable if available
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub_v = self.create_publisher(Float32, topic_v, qos)
        self.pub_p = self.create_publisher(Float32, topic_p, qos)
        self.pub_c = self.create_publisher(Float32, topic_c, qos)

        self.sub = self.create_subscription(BatteryState, src, self.cb, qos)
        self.sub = self.create_subscription(BatteryState, src, self.cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'BatterySplitter listening on {src} -> '
            f'{topic_v}, {topic_p}, {topic_c} '
            f'(percent_scale_0_100={self.scale_pct})'
        )

    def cb(self, msg: BatteryState):
        # Voltage (V)
        if valid(msg.voltage):
            self.pub_v.publish(Float32(data=float(msg.voltage)))

        # Current (A) — in BatteryState, negative typically means discharging
        if valid(msg.current):
            self.pub_c.publish(Float32(data=float(msg.current)))

        # Percentage — BatteryState.percentage is 0.0..1.0 (or -1.0 if unknown)
        if valid(msg.percentage) and msg.percentage >= 0.0:
            pct = float(msg.percentage * 100.0) if self.scale_pct else float(msg.percentage)
            self.pub_p.publish(Float32(data=pct))

def main():
    rclpy.init()
    node = BatterySplitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
