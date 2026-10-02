import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_package_directory = get_package_share_directory('turtlebot3_bringup')

    include_launch = IncludeLaunchDescription(
        os.path.join(robot_package_directory, 'launch', 'camera_robot.launch.py')
    )



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
        ),
        include_launch
    ])
