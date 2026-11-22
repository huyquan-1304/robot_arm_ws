#!/usr/bin/env python3
"""Xem URDF xe AMM trong RViz, khong chay Gazebo.

Chay:
  ros2 launch amm_base_description display.launch.py

Thanh truot joint_state_publisher_gui xoay 4 banh de kiem tra chieu truc banh.
"""

from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE_NAME = 'amm_base_description'


def generate_launch_description():
    share = Path(get_package_share_directory(PACKAGE_NAME))
    xacro_file = share / 'urdf' / 'amm_base.urdf.xacro'
    rviz_file = share / 'rviz' / 'amm_base.rviz'

    robot_description = xacro.process_file(str(xacro_file)).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(rviz_file), '-f', 'base_footprint'],
            output='screen',
        ),
    ])
