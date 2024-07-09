import rclpy
from rclpy.node import Node


from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu


class Controller(Node):

    def __init__(self):
        super().__init__("controller")
        # self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 10
        self.get_logger().info("node created")

        self.laser_sub = self.create_subscription(
            LaserScan, "/lidar", self.laser_callback, 10
        )
        self.imu_sub = self.create_subscription(
            LaserScan, "/imu", self.imu_callback, 10
        )

    def laser_callback(self, data: LaserScan):
        self.get_logger().info("lidar_callback:")
        self.get_logger().info(str(data.ranges))

    def imu_callback(self, data: Imu):
        self.get_logger().info("imu callback:")
        self.get_logger().info(str(data.linear_acceleration))

    def cmd_vel_callback():
        pass


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    # rclpy.spin(controller)
    rclpy.spin_once(controller)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
