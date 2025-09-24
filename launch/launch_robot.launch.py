# rosmower/launch/main.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

# Full device path for hoverboard Arduino
BY_ID = '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0'

# RPLIDAR device path
RPLIDAR_BY_ID = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"

# Default FCU path for MAVROS (override at launch with: `ros2 launch ... dev:=/dev/ttyACM0`)
DEFAULT_FCU = '/dev/ttyACM0'

def generate_launch_description():
    pkg = get_package_share_directory('rosmower')

    # --- Launch arguments ---
    use_sim_time       = LaunchConfiguration('use_sim_time')
    use_ros2_control   = LaunchConfiguration('use_ros2_control')
    use_rosbridge      = LaunchConfiguration('use_rosbridge')
    use_twist_mux      = LaunchConfiguration('use_twist_mux')
    use_mavros         = LaunchConfiguration('use_mavros')
    dev                = LaunchConfiguration('dev')  # MAVROS serial device
    arm                = LaunchConfiguration('arm')  # Explicit flag to arm motors
    use_joint_state_gui = LaunchConfiguration('use_joint_state_gui')  # Toggle GUI sliders

    # Build MAVROS fcu_url = serial://<dev>:115200
    fcu_url = ['serial://', dev, ':115200']

    # --- Global: keep console quiet ---
    quiet_env = SetEnvironmentVariable('RCUTILS_LOGGING_SEVERITY_THRESHOLD', 'WARN')

    # --- Core robot state publisher ---
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': use_ros2_control,
        }.items(),
    )

    # --- Joint State Publisher (only when NOT using ros2_control) ---
    # CLI sliders:
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        condition=IfCondition(use_joint_state_gui)
    )
    # Headless publisher:
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        condition=UnlessCondition(use_joint_state_gui)
    )
    # Run either GUI or headless, but only if ros2_control is disabled
    jsp_group = GroupAction(
        actions=[jsp_gui, jsp],
        condition=UnlessCondition(use_ros2_control)
    )

    # --- Optional: Twist Mux support ---
    twist_mux_params = os.path.join(pkg, 'config', 'twist_mux.yaml')
    twist_mux_ctrl = Node(
        package='twist_mux',
        executable='twist_mux',
        output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
    )
    twist_mux_bridge = Node(
        package='twist_mux',
        executable='twist_mux',
        output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
    )
    twist_mux_to_controller = GroupAction(actions=[twist_mux_ctrl], condition=IfCondition(use_ros2_control))
    twist_mux_to_bridge = GroupAction(actions=[twist_mux_bridge], condition=IfCondition(use_twist_mux))

    # --- Hoverboard bridge (only starts if --arm flag is passed) ---
    hoverboard = Node(
        package='rosmower',
        executable='hoverboard_bridge_node.py',
        name='hoverboard_bridge',
        output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[{
            'port': BY_ID,
            'baud': 115200,
            'max_pwm': 25,
            'max_lin': 1.0,
            'max_ang': 2.0,
            'stat_period': 0.5,
            'arm_on_start': True,
        }],
        condition=IfCondition(arm)
    )
    hoverboard_group = GroupAction(actions=[hoverboard], condition=UnlessCondition(use_ros2_control))
    delayed_hoverboard = TimerAction(period=3.0, actions=[hoverboard_group])

    # --- IMU Bridge Node ---
    imu_bridge =  Node(
        package='rosmower',
        executable='imu_bridge.py',
        name='imu_bridge',
        output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
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
        output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[ekf_config],
        remappings=[('/odometry/filtered', '/odom')],
    )

    # --- Optional: rosbridge websocket ---
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[{'port': 9090}],
        condition=IfCondition(use_rosbridge),
    )

    # --- RPLIDAR motor control node ---
    rplidar_motor_control = Node(
        package='rosmower',
        executable='rplidar_motor_control.py',
        name='rplidar_motor_control',
        output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[{
            'topic_name': '/scan',
            'node1': 'rviz',
            'node2': 'move_base',
            'seconds_between_tries': 5,
        }],
    )

    # --- RPLIDAR driver node ---
    rplidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='rplidar',
        output='log',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[{
            'serial_port': RPLIDAR_BY_ID,
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'angle_compensate': True,
        }],
    )

    # --- Static transform from laser to base_link ---
    rplidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_to_base_tf',
        output='log',
        arguments=['0.20','0','0.25','0','0','0','base_link','laser_frame', '--ros-args', '--log-level', 'warn']
    )

    delayed_rplidar = TimerAction(period=3.0, actions=[rplidar_motor_control, rplidar_node, rplidar_tf])

    # --- MAVROS Node ---
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[{
            'fcu_url': fcu_url,
            'fcu_protocol': 'v2.0',
        }],
        condition=IfCondition(use_mavros),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_ros2_control', default_value='false'),
        DeclareLaunchArgument('use_twist_mux', default_value='false'),
        DeclareLaunchArgument('use_rosbridge', default_value='true'),
        DeclareLaunchArgument('use_mavros', default_value='true'),
        DeclareLaunchArgument('use_joint_state_gui', default_value='true',
                              description='Use joint_state_publisher_gui sliders'),
        DeclareLaunchArgument('dev', default_value=DEFAULT_FCU),
        DeclareLaunchArgument('arm', default_value='false', description='Enable motor arming if true'),

        quiet_env,
        rsp,
        jsp_group,            # <-- NEW: joint state publisher (headless or GUI)
        #twist_mux_to_controller,
        #twist_mux_to_bridge,
        delayed_hoverboard,
        imu_bridge,
        ekf_node,
        delayed_rplidar,
        rosbridge_node,
        mavros_node,
    ])
