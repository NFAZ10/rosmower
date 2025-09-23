from launch import LaunchDescription
from launch_ros.actions import Node

BY_ID = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': BY_ID,
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                # Optional tuning:
                # 'scan_mode': 'Sensitivity',  # or 'Standard'
                # 'scan_frequency': 10.0,
                # 'max_distance': 12.0,
            }],
        ),
    ])
