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
            # # Start mode change client with parameter (default to 0, change to 1 if needed)
            # ExecuteProcess(
            #     cmd=['ros2', 'run', 'overlord100_mode_switcher', 'change_mode_client', '0'],  # Change '0' to '1' if needed
            #     output='screen'
            # ),
            
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
        ]
    )
