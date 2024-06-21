import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # pkg_share = launch_ros.substitutions.FindPackageShare(package="urdf_dummy").find("urdf_dummy")
    # world_path = os.path.join(pkg_share, "worlds/dummy_world.sdf")
    # model_path = os.path.join(pkg_share, "description/dummy_bot.urdf")
    # print(world_path)
    # swpan_args = '{sdf_filename: "' + world_path + '" , name: "my_robot"}'
    # return LaunchDescription(
    #     [
    #         ExecuteProcess(
    #             cmd=[
    #                 "ign",
    #                 "service",
    #                 "-s",
    #                 "/world/dummy/create",
    #                 "--reqtype",
    #                 "ignition.msgs.EntityFactory",
    #                 "--reptype",
    #                 "ignition.msgs.Boolean",
    #                 "--timeout",
    #                 "1000",
    #                 "--req",
    #                 ""
    #                 'name: "test"; sdf_filename:"home/ws/install/urdf_dummy/share/urdf_dummy/description/dummy_bot.urdf"; pose: {position: {y: -5 z: 1}}',
    #             ],
    #             output="screen",
    #         ),
    #     ]
    # )

    ignition_spawn_entity = Node(
        package="ros_ign_gazebo",
        executable="create",
        output="screen",
        arguments=[
            "-entity",
            TURTLEBOT3_MODEL,
            "-name",
            TURTLEBOT3_MODEL,
            "-file",
            PathJoinSubstitution(
                [
                    get_package_share_directory("turtlebot3"),
                    "models",
                    "turtlebot3",
                    "model.sdf",
                ]
            ),
            "-allow_renaming",
            "true",
            "-x",
            "-2.0",
            "-y",
            "-0.5",
            "-z",
            "0.01",
        ],
    )
