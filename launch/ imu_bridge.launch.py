from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosmower',
            executable='imu_bridge',
            name='imu_bridge',
            output='screen',
            parameters=[{
                'mavros_imu_topic': '/mavros/imu/data',
                'output_imu_topic': '/imu/data_raw'
            }]
        )
    ])
