from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_gravity_comp',
            default_value='true',
            description='Enable WAM gravity compensation automatically after startup.',
        ),
        DeclareLaunchArgument(
            'initialize_hand_on_startup',
            default_value='false',
            description='Initialize the BarrettHand automatically after startup.',
        ),
        DeclareLaunchArgument(
            'hand_clearance_move_on_startup',
            default_value='false',
            description='Move WAM joint 4 before automatic hand initialization.',
        ),
        DeclareLaunchArgument(
            'home_velocity',
            default_value='0.10',
            description='Joint-space velocity used by /wam/go_home.',
        ),
        DeclareLaunchArgument(
            'home_acceleration',
            default_value='0.10',
            description='Joint-space acceleration used by /wam/go_home.',
        ),
        Node(
            package='wam_node',
            executable='wam_node',
            name='wam_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'auto_gravity_comp': LaunchConfiguration('auto_gravity_comp'),
                'initialize_hand_on_startup': LaunchConfiguration('initialize_hand_on_startup'),
                'hand_clearance_move_on_startup': LaunchConfiguration('hand_clearance_move_on_startup'),
                'home_velocity': LaunchConfiguration('home_velocity'),
                'home_acceleration': LaunchConfiguration('home_acceleration'),
            }],
        ),
    ])
