import rclpy
from rclpy.node import Node


from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan


class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.laser_sub = self.create_subscription(LaserScan, "/lidar", self. laser_callback, 10 )
        

    def laser_callback(self, data : LaserScan):
        self.get_logger().info(data.ranges)
    

    



def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(contoller)

    
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()