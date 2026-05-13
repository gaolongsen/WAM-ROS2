from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='barrett_hand_node',
            executable='barrett_hand_node',
            name='barrett_hand_node',
            output='screen',
        ),
    ])
