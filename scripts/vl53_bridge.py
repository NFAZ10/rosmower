#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

PORT = '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usbv2-0:1.1.1:1.0'   # or COMx on Windows; adjust to your board
BAUD = 115200

class VL53Bridge(Node):
    def __init__(self):
        super().__init__('vl53_bridge')
        self.pub = self.create_publisher(String, 'vl53_distances', 10)
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=1)
            self.get_logger().info(f'Opened {PORT} at {BAUD} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial: {e}')
            raise
        self.timer = self.create_timer(0.1, self.read_line)

    def read_line(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line.startswith('dist_mm:'):
                msg = String()
                msg.data = line[len('dist_mm:'):]
                self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VL53Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
