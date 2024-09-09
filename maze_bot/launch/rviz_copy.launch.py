import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
        package='maze_bot',
        executable='talker_node',
        name='Publisher',
        output='screen'),

        Node(
        package='maze_bot',
        executable='listener_node',
        name='Subscriber',
        output='screen'),

    ])