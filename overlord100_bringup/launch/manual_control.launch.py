import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Launch URDF simulation
            ExecuteProcess(
                cmd=["ros2", "launch", "urdf_dummy", "sim_custom_controller.launch.py"],
                output="screen",
            ),
            # Start diff_drive_controller node
            Node(
                package="overlord100_controller",
                executable="diff_drive_controller",
                name="diff_drive_controller",
                output="screen",
            ),
            # Start mode_switcher node
            Node(
                package="overlord100_mode_switcher",
                executable="mode_switcher",
                name="mode_switcher",
                output="screen",
            ),
            # Start log_collector node
            Node(
                package="overlord100_logger",
                executable="log_collector",
                name="log_collector",
                output="screen",
            ),
            # Start ROSBridge server
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    "rosbridge_server",
                    "rosbridge_websocket_launch.xml",
                ],
                output="screen",
            ),
            # Launch SLAM
            ExecuteProcess(
                cmd=["ros2", "launch", "overlord100_slam", "slam_launch.launch.py"],
                output="screen",
            ),
        ]
    )
