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
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable
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
            "3",
        ],
    )

    # Spawn world
    ignition_spawn_world = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file",
            PathJoinSubstitution(
                [get_package_share_directory("urdf_dummy"), "models", world_name]
            ),
            "-allow_renaming",
            "false",
        ],
    )
    set_env_vars_resources = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', pack_dir + '/models' )

    print(pack_dir)
    

    

    return LaunchDescription(
        
        [   
            
            set_env_vars_resources,
            ExecuteProcess(
            cmd=['python3', pack_dir+'/launch/test.py'],
            output='screen'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                get_package_share_directory("ros_gz_sim"),
                                "launch",
                                "gz_sim.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments=[("ign_args", [" -r -v 3"])],
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
