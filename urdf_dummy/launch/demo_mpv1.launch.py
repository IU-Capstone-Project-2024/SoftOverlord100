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
                "/sim_no_controller.launch.py",
            ]
        ),
    )

    diff_drive_controller = Node(
        package="overlord100_controller",
        executable="diff_drive_controller",
    )
    
    rqt_robot_steering = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
    )

    return LaunchDescription(
        [
            spawn_models_node,
            diff_drive_controller,
            rqt_robot_steering,
        ]
    )
