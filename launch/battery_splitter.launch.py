from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosmower',
            executable='battery_splitter.py',
            name='battery_splitter',
            output='screen',
            parameters=[{
                'source_topic': '/mavros/battery',   # BatteryState
                'voltage_topic': '/voltage',         # Float32 (V)
                'percent_topic': '/percent',         # Float32 (% 0..100)
                'current_topic': '/current',         # Float32 (A, signed)
                'percent_scale_0_100': True
            }]
        )
    ])
