from launch import LaunchDescription
from launch_ros.actions import Node

BY_ID = '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosmower',
            executable='hoverboard_bridge_node.py',
            name='hoverboard_bridge',
            output='screen',
            parameters=[{
                 'port': BY_ID,
                 'baud': 115200,
                 'max_pwm': 255,
                 'max_lin': 1.0,
                 'max_ang': 2.0,
                 'stat_period': 0.5,
                 'arm_on_start': True,
            }]
        )
    ])
