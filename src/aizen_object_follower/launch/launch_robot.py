from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aizen_object_follower',
            executable='find_object',
            name='find_object',
        ),
        Node(
            package='aizen_object_follower',
            executable='get_object_distance',
            name='get_object_distance',
        ),
        Node(
            package='aizen_object_follower',
            executable='chasedown',
            name='chasedown'
        )
    ])
