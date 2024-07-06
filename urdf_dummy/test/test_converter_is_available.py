import pytest
import unittest

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

from overlord100_msgs.msg import WheelsData

from ament_index_python.packages import get_package_share_directory



import os
import sys
import time
import unittest
import uuid

import rclpy


@pytest.mark.rostest
def generate_test_description():


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

    
    
    converter = Node(
        package="urdf_dummy",
        executable="converter"
    )

    return (LaunchDescription(
        [
            spawn_models_node,
            bridge_setup_node,
            robot_state_publisher_node,
            transforms,
            converter,
            
            
            ReadyToTest()
        ]
    ),{
        "converter":converter,
    }
    )






class TestSimulationTopics(unittest.TestCase):

    
    @classmethod
    def setUpClass(cls) -> None:
      rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()


    def setUp(self):
        self.node = rclpy.create_node("test_simulation_topics")

    def tearDown(self):
        self.node.destroy_node()

    def test_converter_lidars_data_received(self):
        messages = []
        wheels = self.node.create_subscription(LaserScan, 
            "/laser_scan_front",
            lambda response: messages.append(response),
            10

        )

        try:
            end_time = time.time() + 20
            while time.time()< end_time:
                rclpy.spin_once(self.node, timeout_sec= 0.1)
                if len(messages) > 2:
                    break

            self.assertGreater(len(messages), 2)
            
        finally:
            self.node.destroy_subscription(wheels)        

    def test_converter_color_camera_data_received(self):
        messages = []
        test_subscription = self.node.create_subscription(Image, 
            "/color_camera",
            lambda response: messages.append(response),
            10

        )

        try:
            end_time = time.time() + 20
            while time.time()< end_time:
                rclpy.spin_once(self.node, timeout_sec= 0.1)
                if len(messages) > 2:
                    break

            self.assertGreater(len(messages), 2)
            
        finally:
            self.node.destroy_subscription(test_subscription)        

    def test_converter_depth_camera_data_received(self):
        messages = []
        test_subscription = self.node.create_subscription(PointCloud2, 
            "/depth_camera/points",
            lambda response: messages.append(response),
            10

        )

        try:
            end_time = time.time() + 20
            while time.time()< end_time:
                rclpy.spin_once(self.node, timeout_sec= 0.1)
                if len(messages) > 2:
                    break

            self.assertGreater(len(messages), 2)
            
        finally:
            self.node.destroy_subscription(test_subscription)        

    def test_converter_tf_data_received(self):
            messages = []
            test_subscription = self.node.create_subscription(TFMessage, 
                "/tf",
                lambda response: messages.append(response),
                10

            )

            try:
                end_time = time.time() + 20
                while time.time()< end_time:
                    rclpy.spin_once(self.node, timeout_sec= 0.1)
                    if len(messages) > 2:
                        break

                self.assertGreater(len(messages), 2)
                
            finally:
                self.node.destroy_subscription(test_subscription)        

    def test_converter_sonar_data_received(self):
            messages = []
            test_subscription = self.node.create_subscription(LaserScan, 
                "/sonar_1_scan",
                lambda response: messages.append(response),
                10

            )

            try:
                end_time = time.time() + 20
                while time.time()< end_time:
                    rclpy.spin_once(self.node, timeout_sec= 0.1)
                    if len(messages) > 2:
                        break

                self.assertGreater(len(messages), 2)
                
            finally:
                self.node.destroy_subscription(test_subscription)        
