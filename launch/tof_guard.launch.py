from launch import LaunchDescription
from launch_ros.actions import Node
import math

def generate_launch_description():
    z = 0.15  # sensor height (m)
    deg = math.pi/180.0

    return LaunchDescription([
        Node(
            package='rosmower',
            executable='tof_guard.py',
            name='tof_guard',
            output='screen',
            parameters=[{
                'xshut_pins': [27, 22, 23, 24, 25],          # BCM
                'addresses':  [0x30, 0x31, 0x32, 0x33, 0x34],

                'frames': ['tof_front_left','tof_front_right','tof_left','tof_right','tof_rear'],
                'topics': ['tof/front_left/range','tof/front_right/range','tof/left/range','tof/right/range','tof/rear/range'],

                'thresholds_cm': [35, 35, 25, 25, 30],
                'poll_hz': 20.0,
                'cmd_vel_in':  '/cmd_vel_in',
                'cmd_vel_out': '/cmd_vel',
                'cooldown_s': 0.4,
                'field_of_view_deg': 25.0,
                'max_range_m': 2.0,
                'min_range_m': 0.03,
            }]
        ),

        # base_link -> tof_front_left (forward +0.30m, left +0.12m, yaw +10°)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_front_left',
            arguments=[ '0.30','0.12',str(z), '0','0',str(+10*deg), 'base_link','tof_front_left' ]
        ),
        # base_link -> tof_front_right (forward +0.30m, right -0.12m, yaw -10°)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_front_right',
            arguments=[ '0.30','-0.12',str(z), '0','0',str(-10*deg), 'base_link','tof_front_right' ]
        ),
        # base_link -> tof_left (left +0.22m, yaw +90°)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_left',
            arguments=[ '0.00','0.22',str(z), '0','0',str(+90*deg), 'base_link','tof_left' ]
        ),
        # base_link -> tof_right (right -0.22m, yaw -90°)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_right',
            arguments=[ '0.00','-0.22',str(z), '0','0',str(-90*deg), 'base_link','tof_right' ]
        ),
        # base_link -> tof_rear (back -0.30m, yaw 180°)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_rear',
            arguments=[ '-0.30','0.00',str(z), '0','0',str(180*deg), 'base_link','tof_rear' ]
        ),
    ])
