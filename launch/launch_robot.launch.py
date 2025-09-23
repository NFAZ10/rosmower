import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

# Full device path for hoverboard Arduino
BY_ID = '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0'

def generate_launch_description():
    pkg = get_package_share_directory('rosmower')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_rosbridge = LaunchConfiguration('use_rosbridge')
    use_twist_mux = LaunchConfiguration('use_twist_mux')

    # --- Core robot state publisher ---
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': use_ros2_control,
        }.items(),
    )

    # --- Optional: Twist Mux support ---
    twist_mux_params = os.path.join(pkg, 'config', 'twist_mux.yaml')
    twist_mux_ctrl = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
    )
    twist_mux_bridge = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
    )
    twist_mux_to_controller = GroupAction(actions=[twist_mux_ctrl], condition=IfCondition(use_ros2_control))
    twist_mux_to_bridge = GroupAction(actions=[twist_mux_bridge], condition=IfCondition(use_twist_mux))

    # --- Hoverboard bridge (when not using ros2_control) ---
    hoverboard = Node(
        package='rosmower',
        executable='hoverboard_bridge_node.py',
        name='hoverboard_bridge',
        output='screen',
        parameters=[{
            'port': BY_ID,
            'baud': 115200,
            'max_pwm': 255,
            'max_lin': 1.0,
            'max_ang': 2.0,
            'stat_period': 0.5,
            'arm_on_start': True,
        }],
    )
    hoverboard_group = GroupAction(actions=[hoverboard], condition=UnlessCondition(use_ros2_control))
    delayed_hoverboard = TimerAction(period=3.0, actions=[hoverboard_group])

    # --- IMU Bridge Node ---
    imu_bridge =  Node(
            package='rosmower',
            executable='imu_bridge.py',
            name='imu_bridge',
            output='screen',
            parameters=[{
                'mavros_imu_topic': '/mavros/imu/data',
                'output_imu_topic': '/imu/data_raw'
            }]
        
    )

    # --- EKF Node (robot_localization) ---
    ekf_config = os.path.join(pkg, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ],
    )

    # --- Optional: rosbridge websocket ---
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}],
        condition=IfCondition(use_rosbridge),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_ros2_control', default_value='false'),
        DeclareLaunchArgument('use_twist_mux', default_value='false'),
        DeclareLaunchArgument('use_rosbridge', default_value='true'),

        rsp,
        twist_mux_to_controller,
        twist_mux_to_bridge,
        delayed_hoverboard,
        imu_bridge,
        ekf_node,
        rosbridge_node,
    ])
