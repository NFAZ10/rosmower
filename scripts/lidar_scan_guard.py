#!/usr/bin/env python3
import rclpy
import signal
import time
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

class LidarScanGuard(Node):
    def __init__(self):
        super().__init__('lidar_scan_guard')

        # Parameters
        self.declare_parameter('start_service', '/sllidar_node/start_scan')
        self.declare_parameter('stop_service',  '/sllidar_node/stop_scan')
        self.declare_parameter('wait_for_hood', True)
        self.declare_parameter('hood_state_topic', '/lidar_hood/state')
        self.declare_parameter('hood_open_value', 'OPEN')
        self.declare_parameter('startup_delay_sec', 0.0)  # optional settle time before starting

        self.start_srv_name = self.get_parameter('start_service').get_parameter_value().string_value
        self.stop_srv_name  = self.get_parameter('stop_service').get_parameter_value().string_value
        self.wait_for_hood  = self.get_parameter('wait_for_hood').get_parameter_value().bool_value
        self.hood_topic     = self.get_parameter('hood_state_topic').get_parameter_value().string_value
        self.hood_open_val  = self.get_parameter('hood_open_value').get_parameter_value().string_value
        self.startup_delay  = self.get_parameter('startup_delay_sec').get_parameter_value().double_value

        self._hood_is_open = not self.wait_for_hood  # if not waiting, treat as already open
        self._started = False

        # Subscribe to hood state if requested
        if self.wait_for_hood:
            self.create_subscription(String, self.hood_topic, self._hood_cb, 10)

        # Prepare service clients
        self.start_cli = self.create_client(Trigger, self.start_srv_name)
        self.stop_cli  = self.create_client(Trigger, self.stop_srv_name)

        # Periodic timer to drive the simple state machine
        self.timer = self.create_timer(0.25, self._tick)

        # Ensure we send stop on SIGINT/SIGTERM too
        signal.signal(signal.SIGINT,  self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self.get_logger().info(f'LidarScanGuard up. start_service={self.start_srv_name}, stop_service={self.stop_srv_name}, wait_for_hood={self.wait_for_hood}')

    def _hood_cb(self, msg: String):
        if msg.data.strip().upper() == self.hood_open_val.upper():
            if not self._hood_is_open:
                self.get_logger().info('Hood is OPEN; ready to start scan.')
            self._hood_is_open = True

    def _wait_for_service(self, cli, name, timeout=10.0):
        t0 = time.time()
        while rclpy.ok() and not cli.wait_for_service(timeout_sec=0.2):
            if time.time() - t0 > timeout:
                return False
        return True

    def _call_trigger(self, cli, name) -> bool:
        if not self._wait_for_service(cli, name, timeout=10.0):
            self.get_logger().warn(f'Service {name} not available.')
            return False
        req = Trigger.Request()
        fut = cli.call_async(req)
        rclpy.task.Future  # keep type checker happy
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if fut.done() and fut.result() and fut.result().success:
            self.get_logger().info(f'{name} OK: {fut.result().message}')
            return True
        self.get_logger().warn(f'{name} failed or no response.')
        return False

    def _tick(self):
        # Start scan once: after hood open (if required) and once services are available
        if not self._started and self._hood_is_open:
            if self.startup_delay > 0:
                self.get_logger().info(f'Waiting {self.startup_delay:.2f}s before starting scan...')
                time.sleep(self.startup_delay)

            ok = self._call_trigger(self.start_cli, self.start_srv_name)
            if ok:
                self._started = True
                self.get_logger().info('LiDAR scanning started (motor spinning).')

    def _signal_handler(self, signum, frame):
        self.get_logger().info(f'Received signal {signum}. Stopping LiDAR scan...')
        try:
            self._call_trigger(self.stop_cli, self.stop_srv_name)
        finally:
            rclpy.shutdown()

def main():
    rclpy.init()
    node = LidarScanGuard()
    try:
        rclpy.spin(node)
    finally:
        # Fallback stop at normal shutdown path
        try:
            node._call_trigger(node.stop_cli, node.stop_srv_name)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
