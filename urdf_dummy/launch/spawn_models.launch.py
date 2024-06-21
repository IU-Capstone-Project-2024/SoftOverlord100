import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    world_name = LaunchConfiguration("world_name", default="dummy_world.sdf")

    # Spawn robot
    ignition_spawn_entity = Node(
        package="ros_ign_gazebo",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "dummy_robot",
            "-file",
            PathJoinSubstitution(
                [
                    get_package_share_directory("urdf_dummy"),
                    "description",
                    "dummy_bot.urdf",
                ]
            ),
            "-allow_renaming",
            "true",
            "-x",
            "-2.0",
            "-y",
            "-0.5",
            "-z",
            "0.5",
        ],
    )

    # Spawn world
    ignition_spawn_world = Node(
        package="ros_ign_gazebo",
        executable="create",
        output="screen",
        arguments=[
            "-file",
            PathJoinSubstitution(
                [get_package_share_directory("urdf_dummy"), "worlds", world_name]
            ),
            "-allow_renaming",
            "false",
        ],
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                get_package_share_directory("ros_ign_gazebo"),
                                "launch",
                                "ign_gazebo.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments=[("ign_args", [" -r"])],
            ),
            ignition_spawn_world,
            ignition_spawn_entity,
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="If true, use simulated clock",
            ),
            DeclareLaunchArgument(
                "world_name", default_value=world_name, description="World name"
            ),
        ]
    )
