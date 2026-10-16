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
            package='aizen_navigate_to_goal',
            executable='find_vector',
            name='find_vector',
        ),
        Node(
            package='aizen_navigate_to_goal',
            executable='find_robot_pos',
            name='find_robot_pos',
        ),
        Node(
            package='aizen_navigate_to_goal',
            executable='onward',
            name='onward',
        )
    ])
