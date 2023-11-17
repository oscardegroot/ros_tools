#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'ros_tools'

    ld = LaunchDescription()

    # Calculate the absolute path to the YAML file
    current_path = os.path.dirname(os.path.abspath(__file__))
    package_path = os.path.join(
        current_path,
        "..")


    node= Node(
        package=package_name,
        executable='example_py',  # Replace with your node's executable name
        name='rostools_example',
        output='screen',
    )
    ld.add_action(node)
    
    node = Node(package='rviz2',
             executable='rviz2',
             arguments=['-d', os.path.join(package_path, 'rviz', 'example_py.rviz')])
    ld.add_action(node)
    
    return ld

