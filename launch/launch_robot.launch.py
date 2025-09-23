import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch.actions import TimerAction
from launch.conditions import IfCondition, UnlessCondition

BY_ID = '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0'

def generate_launch_description():
    pkg = get_package_share_directory('rosmower')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_rosbridge = LaunchConfiguration('use_rosbridge')

    # Include robot_state_publisher (rsp.launch.py processes the URDF)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': use_ros2_control}.items(),
    )

    # twist_mux nodes (conditional). Some setups may not have twist_mux installed,
    # so we gate launching it behind `use_twist_mux`.
    use_twist_mux = LaunchConfiguration('use_twist_mux')
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
    # Only start twist_mux controller bridge if use_twist_mux is true
    twist_mux_to_controller = GroupAction(actions=[twist_mux_ctrl], condition=IfCondition(use_twist_mux))
    twist_mux_to_bridge = GroupAction(actions=[twist_mux_bridge], condition=IfCondition(use_twist_mux))

    # Hoverboard bridge (default when not using ros2_control)
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
    # Start hoverboard after a short delay so other nodes come up first
    delayed_hoverboard = TimerAction(period=3.0, actions=[hoverboard_group])
    # rosbridge websocket (optional)
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}],
        condition=IfCondition(use_rosbridge)
    )

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_ros2_control', default_value='false'),
        DeclareLaunchArgument('use_twist_mux', default_value='false'),
        DeclareLaunchArgument('use_rosbridge', default_value='true'),
    rsp,
    twist_mux_to_controller,
    twist_mux_to_bridge,
    delayed_hoverboard,
    rosbridge_node,
    ])

    return ld