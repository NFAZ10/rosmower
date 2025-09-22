# stability.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosmower',
            executable='stability_controller',
            name='stability_controller',
            parameters=[{
                'cmd_in': '/cmd_vel_raw',
                'cmd_out': '/cmd_vel',
                'use_ekf_yaw': True,
                'yaw_kp': 1.2,
                'yaw_kd': 0.10,
                'max_ang_correction': 0.4,
                'max_lin_accel': 0.7,
                'capture_lin_min': 0.12,
            }],
            output='screen'
        ),
    ])
