import launch
import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    launch_gazebo_arg = IncludeLaunchDescription(
 PythonLaunchDescriptionSource([get_package_share_directory('urdf_dummy'), '/launch', '/gazebo.launch.py']),
 launch_arguments={
  'ign_args' : "empty.sdf"
 }.items(),
)

    spawn_robot_arg = Node(
    package='ros_ign_gazebo',
    executable='create',
    output='screen',
    arguments=["-file", "<path_to_urdf>/dummy_bot.urdf"]
    )

    return LaunchDescription([
    launch_gazebo_arg,
    spawn_robot_arg
    ])