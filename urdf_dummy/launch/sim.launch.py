import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    spawn_models_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("urdf_dummy"), "launch"),
                "/spawn_models.launch.py",
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

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("urdf_dummy"), "launch"),
                "/robot_state_publisher.launch.py",
            ]
        )
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
            spawn_models_node,
            bridge_setup_node,
            rqt_steering_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
