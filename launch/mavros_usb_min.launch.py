from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Default by-id path (replace with your actual device if needed)
DEFAULT_FCU = "/dev/ttyACM0"


def generate_launch_description():
    dev_arg = DeclareLaunchArgument('dev', default_value=DEFAULT_FCU, description='Serial device path for the FCU')
    dev = LaunchConfiguration('dev')

    # Build the fcu_url from substitutions to match the CLI: serial:///${DEV}:115200
    # Use ['serial://', dev, ':115200'] so a dev value like '/dev/ttyACM0' becomes
    # 'serial:///dev/ttyACM0:115200' (3 slashes total).
    fcu_url = ['serial://', dev, ':115200']

    return LaunchDescription([
        dev_arg,
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[{
                'fcu_url': fcu_url,
                'fcu_protocol': 'v2.0',
            }],
        )
    ])
