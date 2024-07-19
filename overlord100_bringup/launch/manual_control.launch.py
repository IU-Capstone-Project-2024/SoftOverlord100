import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription(
        [
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
            # Launch lidars
            # ExecuteProcess(
            #     cmd=["ros2", "launch", "sllidar_ros2", "sllidar_c1_launch.py"],
            #     output="screen",
            # ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("overlord100_bringup"), '/launch', '/lidars.launch.py'])
            )
            # Start ros2socketcan_bridge node
            Node(
                package="ros2socketcan_bridge",
                executable="ros2socketcan",
                name="ros2socketcan",
                output="screen",
            ),
            # Start ros2socketcan_bridge node
            Node(
                package="zlac8015d",
                executable="motors_driver_node",
                name="motors_driver_node",
                output="screen",
            ),
            # Start battery_node
            Node(
                package="ads1115",
                executable="battery_monitor",
                name="battery_monitor",
                output="screen",
            ),
        ]
    )
