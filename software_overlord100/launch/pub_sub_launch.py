from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='software_overlord100',
            executable='publisher',
            name='pub'
        ),
        Node(
            package='software_overlord100',
            executable='subscriber',
            name='sub'
        ),
    ])