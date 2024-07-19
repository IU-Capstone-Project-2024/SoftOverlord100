#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
    lidar_front_config = os.path.join(get_package_share_directory('overlord100_bringup'), '/config', '/lidar_front.yaml')
    lidar_back_config = os.path.join(get_package_share_directory('overlord100_bringup'), '/config', '/lidar_back.yaml')

    return LaunchDescription([
        # Front lidar
        LogInfo("Starting front lidar"),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[lidar_front_config],
            output='screen',
            remappings=[('/scan', '/laser_scan_front')]
            ),

        # Back lidar
        LogInfo("Starting back lidar"),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[lidar_back_config],
            output='screen',
            remappings=[('/scan', '/laser_scan_back')]
            ),
    ])