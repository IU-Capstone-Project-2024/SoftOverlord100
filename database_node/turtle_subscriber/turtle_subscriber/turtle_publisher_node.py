import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from nav_msgs.msg import OccupancyGrid
from pymongo import MongoClient
from geometry_msgs.msg import Quaternion

class TurtlePublisher(Node):
    def __init__(self):
        super().__init__('turtle_publisher')
        
        # MongoDB setup
        self.client = MongoClient('localhost', 27017)
        self.db = self.client['turtle_sim']
        self.pose_collection = self.db['pose']
        self.map_collection = self.db['map']
        
        # ROS publishers
        self.pose_publisher = self.create_publisher(Pose, 'turtle2/pose', 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map2', 10)

        # Timer to periodically publish data
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_data)

        self.get_logger().info('TurtlePublisher node has been started')

    def publish_data(self):
        # Publish the latest pose
        latest_pose = self.pose_collection.find_one(sort=[('_id', -1)])
        if latest_pose:
            pose_msg = Pose()
            pose_msg.x = latest_pose['x']
            pose_msg.y = latest_pose['y']
            pose_msg.theta = latest_pose['theta']
            pose_msg.linear_velocity = latest_pose['linear_velocity']
            pose_msg.angular_velocity = latest_pose['angular_velocity']
            self.pose_publisher.publish(pose_msg)
            self.get_logger().info(f'Published pose: {pose_msg}')
        
        # Publish the latest map
        latest_map = self.map_collection.find_one(sort=[('_id', -1)])
        if latest_map:
            map_msg = OccupancyGrid()
            map_msg.info.width = latest_map['info']['width']
            map_msg.info.height = latest_map['info']['height']
            map_msg.info.resolution = latest_map['info']['resolution']
            map_msg.info.origin.position.x = latest_map['info']['origin']['position']['x']
            map_msg.info.origin.position.y = latest_map['info']['origin']['position']['y']
            map_msg.info.origin.position.z = latest_map['info']['origin']['position']['z']
            
            orientation = latest_map['info']['origin']['orientation']
            map_msg.info.origin.orientation = Quaternion(
                x=orientation['x'],
                y=orientation['y'],
                z=orientation['z'],
                w=orientation['w']
            )
            map_msg.data = latest_map['data']
            self.map_publisher.publish(map_msg)
            self.get_logger().info('Published map data')

def main(args=None):
    rclpy.init(args=args)
    turtle_publisher = TurtlePublisher()
    rclpy.spin(turtle_publisher)
    turtle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
