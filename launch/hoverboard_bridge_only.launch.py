from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosmower',
            executable='hoverboard_bridge_node.py',
            name='hoverboard_bridge',
            output='screen',
            parameters=[{
                 'port': '/dev/ttyUSB0',
                 'baud': 115200,
                 'max_pwm': 255,
                 'max_lin': 1.0,
                 'max_ang': 2.0,
                 'stat_period': 0.5,
                 'arm_on_start': True,
            }]
        )
    ])
