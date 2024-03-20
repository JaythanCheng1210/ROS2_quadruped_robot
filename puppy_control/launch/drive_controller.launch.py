import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='puppy_control',
            executable='imu',
            name='imu',
            output='screen'),
        Node(
            package='puppy_control',
            executable='puppy_control',
            name='puppy_control',
            output='screen'),

    ])