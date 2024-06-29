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
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/laser_scan_back@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/laser_scan_front@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/sonar_1@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/sonar_2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/sonar_3@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/sonar_4@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/sonar_5@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/sonar_6@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/sonar_7@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/sonar_8@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/world/default/model/dummy_robot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",
            "/model/dummy_robot/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/odom/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",

        ],
        remappings=[
            ("/odom/tf", "/tf"),
            ("/world/default/model/dummy_robot/joint_state", "/joint_states"),
            ('/camera_info', '/depth_camera_info')
        ],
        output="screen",
    )

    return LaunchDescription([bridge])
