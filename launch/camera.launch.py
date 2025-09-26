from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_camera_node(context, *args, **kwargs):
    # Convert LaunchConfiguration (strings) to proper Python types
    video_device = LaunchConfiguration('video_device').perform(context)
    width = int(LaunchConfiguration('width').perform(context))
    height = int(LaunchConfiguration('height').perform(context))
    fps = int(LaunchConfiguration('fps').perform(context))
    fmt = LaunchConfiguration('pixel_format').perform(context)
    frame = LaunchConfiguration('frame_id').perform(context)

    v4l2 = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'video_device': video_device,
            'image_size': [width, height],
            'time_per_frame': [1, fps],      # 1/fps
           # 'pixel_format': mjpg,             # MJPG or YUYV
            'output_encoding': 'bgr8',       # common OpenCV encoding; convert if needed
            'camera_frame_id': frame
        }]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            '0', '0', '0',                  # x y z
            '-0.5', '0.5', '-0.5', '0.5',   # qx qy qz qw  (RPY -90, 0, -90)
            'camera_link', 'camera_link_optical'
        ]
    )

    return [v4l2, static_tf]


def generate_launch_description():
    dev_arg = DeclareLaunchArgument('video_device', default_value='/dev/video0')
    # Use device-native safe defaults to avoid MJPG decode problems
    widthArg = DeclareLaunchArgument('width', default_value='640')
    heightArg = DeclareLaunchArgument('height', default_value='480')
    fpsArg = DeclareLaunchArgument('fps', default_value='30')
    fmtArg = DeclareLaunchArgument('pixel_format', default_value='YUYV')  # prefer raw YUYV to avoid MJPG decoding
    frameArg = DeclareLaunchArgument('frame_id', default_value='camera_link_optical')

    return LaunchDescription([dev_arg, widthArg, heightArg, fpsArg, fmtArg, frameArg,
                             OpaqueFunction(function=_build_camera_node)])
