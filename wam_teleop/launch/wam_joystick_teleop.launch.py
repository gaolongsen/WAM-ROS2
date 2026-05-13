from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.12,
            }],
            output='screen',
        ),
        Node(
            package='wam_teleop',
            executable='wam_joystick_teleop',
            name='wam_joystick_teleop',
            output='screen',
        ),
    ])
