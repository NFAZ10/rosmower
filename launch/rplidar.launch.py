# rosmower/launch/lidar_with_guard.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

BY_ID = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"

def generate_launch_description():
    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'serial_port': BY_ID,
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            # optional: 'scan_mode': 'Sensitivity'
        }],
    )

    guard = Node(
        package='rosmower',
        executable='lidar_scan_guard',
        name='lidar_scan_guard',
        output='screen',
        parameters=[{
            # service names can be changed here if your node name differs
            'start_service': '/sllidar_node/start_scan',
            'stop_service':  '/sllidar_node/stop_scan',
            # wait for your hood controller (set to False if you don’t need it)
            'wait_for_hood': False,
            'hood_state_topic': '/lidar_hood/state',
            'hood_open_value': 'OPEN',
            'startup_delay_sec': 0.25,  # small settle before starting
        }],
    )

    return LaunchDescription([lidar, guard])
