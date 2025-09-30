# vl53_bridge.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port_arg   = DeclareLaunchArgument('port',   default_value='/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usbv2-0:1.1.1:1.0')
    baud_arg   = DeclareLaunchArgument('baud',   default_value='115200')
    topic_arg  = DeclareLaunchArgument('topic',  default_value='vl53_distances')
    period_arg = DeclareLaunchArgument('period', default_value='0.10')  # seconds

    return LaunchDescription([
        port_arg, baud_arg, topic_arg, period_arg,
        Node(
            package='rosmower',          # <-- change to your package
            executable='vl53_bridge.py',         # <-- if using console_scripts entry point
            # executable='vl53_bridge.py',    # <-- if launching a script file
            name='vl53_bridge',
            output='screen',
            parameters=[{
                'port':   LaunchConfiguration('port'),
                'baud':   LaunchConfiguration('baud'),
                'topic':  LaunchConfiguration('topic'),
                'period': LaunchConfiguration('period'),
            }],
        )
    ])
