import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class DummyMapPublisher(Node):
    def __init__(self):
        super().__init__('dummy_map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = OccupancyGrid()
        msg.info.width = 10
        msg.info.height = 10
        msg.info.resolution = 1.0
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = [0] * 100
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing dummy map data')

def main(args=None):
    rclpy.init(args=args)
    dummy_map_publisher = DummyMapPublisher()
    rclpy.spin(dummy_map_publisher)
    dummy_map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
