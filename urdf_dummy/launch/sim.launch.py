import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    world_spawner_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("urdf_dummy"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
        launch_arguments={"target_frame": "carrot1"}.items(),
    )

    model_spawner_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("urdf_dummy"), "launch"),
                "/put_urdf.launch.py",
            ]
        ),
    )

    bridge_setup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("urdf_dummy"), "launch"),
                "/setup_bridges.launch.py",
            ]
        ),
    )

    rqt_steering_node = Node(
        package="rqt_robot_steering",
        namespace="rqt_robot_steering",
        executable="rqt_robot_steering",
        name="rqt_robot_steering",
    )

    rviz_node = Node(
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            [
                os.path.join(
                    get_package_share_directory("urdf_dummy"),
                    "rviz",
                    "simulator.rviz",
                )
            ],
        ],
    )
    return LaunchDescription(
        [
            world_spawner_node,
            model_spawner_node,
            bridge_setup_node,
            rqt_steering_node,
            rviz_node,
        ]
    )
