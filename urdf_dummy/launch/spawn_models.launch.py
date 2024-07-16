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
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    AppendEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    world_name = LaunchConfiguration("world_name", default="mvp_world.sdf")
    pack_dir = get_package_share_directory("urdf_dummy")
    # Spawn robot
    ignition_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "overlord100",
            "-file",
            PathJoinSubstitution(
                [
                    get_package_share_directory("urdf_dummy"),
                    "description",
                    "overlord100.urdf",
                ]
            ),
            "-allow_renaming",
            "true",
            "-x",
            "-2.0",
            "-y",
            "-0.5",
            "-z",
            "1",
        ],
    )

    # Spawn world using sdf with world tag
    start_world = ExecuteProcess(
        cmd=[
            "ign",
            "gazebo",
            "-v 4",
            "-r",
            PathJoinSubstitution(
                [
                    get_package_share_directory("urdf_dummy"),
                    "worlds",
                    "mvp_world.sdf",
                ]
            ),
        ]
    )

    def find(name, path):
        for root, dirs, files in os.walk(path):
            if name in dirs:
                return os.path.join(root, name)

    model_path = find("models", os.getenv("PWD"))
    print(model_path)
    set_env_vars_resources = AppendEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH", model_path
    )

    return LaunchDescription(
        [
            set_env_vars_resources,
            start_world,
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
