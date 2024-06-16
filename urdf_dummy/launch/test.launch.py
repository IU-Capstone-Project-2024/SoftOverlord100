import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package='my_first_pkg').find('my_first_pkg')
    default_model_path = os.path.join(pkg_share, 'urdf/my_robot.urdf')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', 'empty.sdf'],
            output='screen'),
    ])