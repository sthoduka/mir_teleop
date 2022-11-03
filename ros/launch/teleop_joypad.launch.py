#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import os
import xacro


def generate_launch_description():

    joypad_config_path = os.path.join(
        get_package_share_directory('mir_teleop'),
        'ros', 'config',
        'teleop.yaml')
    joy_config_path = os.path.join(
        get_package_share_directory('mir_teleop'),
        'ros', 'config',
        'joy.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joy_config_path]
    )

    mir_teleop_node = Node(
            package='mir_teleop',
            executable='teleop_joypad_node',
            name='mir_teleop_joypad',
            output='screen',
            parameters=[joypad_config_path]
#            remappings=[('cmd_vel', '/cmd_vel')]
    )

    return LaunchDescription([
        joy_node,
        mir_teleop_node
    ])
