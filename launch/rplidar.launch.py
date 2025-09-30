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
                # Accuracy improvements:
                'scan_mode': 'Sensitivity',  # Use high sensitivity mode
                'scan_frequency': 5.0,       # Lower frequency for more samples per rotation
                'max_distance': 5.0,
                'ignore_array': '',          # Remove any ignored angles
                'auto_standby': False,       # Keep LIDAR spinning consistently
                # Quality filtering:
                'min_distance': 0.05,        # Filter out very close readings (noise)
                'angle_excludemin': -180.0,  # Include full 360° range
                'angle_excludemax': 180.0,
                'auto_standby': False,
            }],
        ),
         Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_base_tf',
            arguments=['0.20','0','0.25','0','0','0','base_link','laser_frame']
        ),
    ])

