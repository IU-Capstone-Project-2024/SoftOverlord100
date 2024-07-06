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
                "/spawn_model_with_diffdrive.launch.py",
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

    transforms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("urdf_dummy"), "launch"),
                "/static_transforms.launch.py",
            ]
        )
    )
    converter = Node(package="urdf_dummy", executable="converter")

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

    rqt_node = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
        remappings=[
            ("/cmd_vel", "/regular_driver"),
        ],
    )

    return LaunchDescription(
        [spawn_models_node, bridge_setup_node, transforms, robot_state_publisher_node, converter, rviz_node, rqt_node]
    )
