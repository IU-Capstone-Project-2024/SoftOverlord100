import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    
    # Bridge
    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
        output='screen'
    )
    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU"'],
        output='screen'
    )

    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/dummy_bot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
        output='screen'
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/world/dummy/model/test/joint_state@'
            'sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/test/pose@'
            'tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'

        ],
        remappings=[
            ('/model/test/pose', '/tf'),
            ('/world/dummy/model/test/joint_state', '/joint_states')
        ],
        
        output="screen"
    )



    return LaunchDescription([

        bridge
    ])