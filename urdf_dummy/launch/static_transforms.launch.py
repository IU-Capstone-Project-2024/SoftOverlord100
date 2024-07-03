import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():


    static_tf_node_lidar_back = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "lidar_back",
            "overlord100/chassis/lidar_sensor_back",
        ],
    )


    static_tf_node_lidar_front = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "lidar_front",
            "overlord100/chassis/lidar_sensor_front",
        ],
    )

    static_tf_node_realsense = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "realsense",
            "overlord100/chassis/d435_depth",
        ],
    )

    static_tf_node_sonar_1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "sonar1",
            "overlord100/chassis/sonar_1",
        ],
    )

    static_tf_node_sonar_2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "sonar2",
            "overlord100/chassis/sonar_2",
        ],
    )
    static_tf_node_sonar_2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "sonar2",
            "overlord100/chassis/sonar_2",
        ],
    )
    static_tf_node_sonar_3 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "sonar3",
            "overlord100/chassis/sonar_3",
        ],
    )
    static_tf_node_sonar_4 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "sonar4",
            "overlord100/chassis/sonar_4",
        ],
    )
    static_tf_node_sonar_5 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "sonar5",
            "overlord100/chassis/sonar_5",
        ],
    )
    static_tf_node_sonar_6 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "sonar6",
            "overlord100/chassis/sonar_6",
        ],
    )
    static_tf_node_sonar_7 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "sonar7",
            "overlord100/chassis/sonar_7",
        ],
    )
    static_tf_node_sonar_8 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "sonar8",
            "overlord100/chassis/sonar_8",
        ],
    )


   
    
    return LaunchDescription(
        [
           
            static_tf_node_realsense, 
            static_tf_node_lidar_back,
            static_tf_node_lidar_front,

            static_tf_node_sonar_1,
            static_tf_node_sonar_2,
            static_tf_node_sonar_3,
            static_tf_node_sonar_4,
            static_tf_node_sonar_5,
            static_tf_node_sonar_6,
            static_tf_node_sonar_7,
            static_tf_node_sonar_8

           
        ]
    )
