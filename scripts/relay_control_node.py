#!/usr/bin/env python3
import atexit
import rclpy
from rclpy.node import Node
import gpiod
from std_srvs.srv import SetBool

class RelayNode(Node):
    def __init__(self):
        super().__init__('relay_control')
        self.declare_parameter('chip', 'gpiochip0')
        self.declare_parameter('line', 17)
        self.declare_parameter('active_high', True)
        self.declare_parameter('relay_on_start', True)

        chip_name = self.get_parameter('chip').get_parameter_value().string_value
        line_num  = self.get_parameter('line').get_parameter_value().integer_value
        self.active_high = self.get_parameter('active_high').get_parameter_value().bool_value

        self.chip = gpiod.Chip(chip_name)
        self.line = self.chip.get_line(line_num)
        config = gpiod.LineRequest()
        config.consumer = 'relay_control'
        config.request_type = gpiod.LineRequest.DIRECTION_OUTPUT
        self.line.request(config)

        # Ensure OFF on exit
        atexit.register(self._off)

        if self.get_parameter('relay_on_start').get_parameter_value().bool_value:
            self._on()
        else:
            self._off()

        self.srv = self.create_service(SetBool, 'set_relay', self.handle_set_relay)

    def _on(self):
        self.line.set_value(1 if self.active_high else 0)

    def _off(self):
        try:
            self.line.set_value(0 if self.active_high else 1)
        except Exception:
            pass  # ignore if already released

    def handle_set_relay(self, req, res):
        if req.data:
            self._on()
            res.success = True
            res.message = 'Relay ON'
        else:
            self._off()
            res.success = True
            res.message = 'Relay OFF'
        return res

def main():
    rclpy.init()
    node = RelayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
