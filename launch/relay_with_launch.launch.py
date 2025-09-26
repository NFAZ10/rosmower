# file: relay_with_node.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    relay = Node(
        package='your_pkg',              # replace with your package
        executable='relay_control_node.py',
        name='relay_control',
        output='screen',
        parameters=[{
            'chip': 'gpiochip0',
            'line': 17,
            'active_high': True,
            'relay_on_start': True
        }]
    )

    # your other nodes here...
    return LaunchDescription([relay])
