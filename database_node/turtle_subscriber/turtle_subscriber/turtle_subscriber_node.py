import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from nav_msgs.msg import OccupancyGrid
from pymongo import MongoClient

class TurtleSubscriber(Node):
    def __init__(self):
        super().__init__('turtle_subscriber')
        
        self.pose_subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)
        
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
        
        # MongoDB setup
        self.client = MongoClient('localhost', 27017)
        self.db = self.client['turtle_sim']
        self.pose_collection = self.db['pose']
        self.map_collection = self.db['map']

        self.get_logger().info('TurtleSubscriber node has been started')
        
        # Log subscription status
        self.get_logger().info('Subscribed to turtle1/pose')
        self.get_logger().info('Subscribed to map')

    def pose_callback(self, msg):
        #self.get_logger().info(f'Received pose data: {msg}')
        pose_data = {
            'x': msg.x,
            'y': msg.y,
            'theta': msg.theta,
            'linear_velocity': msg.linear_velocity,
            'angular_velocity': msg.angular_velocity,
        }
        self.pose_collection.insert_one(pose_data)
        #self.get_logger().info(f'Inserted pose data: {pose_data}')

    def map_callback(self, msg):
        self.get_logger().info(f'Received map data: {msg}')
        map_data = {
            'info': {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin': {
                    'position': {
                        'x': msg.info.origin.position.x,
                        'y': msg.info.origin.position.y,
                        'z': msg.info.origin.position.z,
                    },
                    'orientation': {
                        'x': msg.info.origin.orientation.x,
                        'y': msg.info.origin.orientation.y,
                        'z': msg.info.origin.orientation.z,
                        'w': msg.info.origin.orientation.w,
                    },
                },
            },
            'data': list(msg.data),
        }
        self.map_collection.insert_one(map_data)
        self.get_logger().info('Inserted map data into MongoDB')

def main(args=None):
    rclpy.init(args=args)
    turtle_subscriber = TurtleSubscriber()
    rclpy.spin(turtle_subscriber)
    turtle_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
