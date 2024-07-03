import rclpy
import math
from rclpy.node import Node
from overlord100_msgs.msg import WheelsData
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan


class Converter(Node):
    def __init__(self):
        super().__init__("converter")
        self.wheels_sub = self.create_subscription(
            WheelsData, "/wheels_control", self.wheels_callback, 10
        )
        self.left_wheel = self.create_publisher(Float64, "/left_wheel", 10)
        self.right_wheel = self.create_publisher(Float64, "/right_wheel", 10)
        self.states = self.create_subscription(
            JointState, "/joint_states", self.encoders_callback, 10
        )
        self.encoders_pub = self.create_publisher(WheelsData, "/wheels_encoders", 10)



        self.sonars_publishers = []
        
        self.sonar_1 = self.create_subscription(LaserScan, "/sonar_1_scan", self.sonar_callback,10 )
        self.sonar_1_pub = self.create_publisher(Range, "/sonar_1", 10)
        self.sonars_publishers.append(self.sonar_1_pub)
        
        self.sonar_2 = self.create_subscription(LaserScan, "/sonar_2_scan", self.sonar_callback,10 )
        self.sonar_2_pub = self.create_publisher(Range, "/sonar_2", 10)
        self.sonars_publishers.append(self.sonar_2_pub)

        self.sonar_3 = self.create_subscription(LaserScan, "/sonar_3_scan", self.sonar_callback,10 )
        self.sonar_3_pub = self.create_publisher(Range, "/sonar_3", 10)
        self.sonars_publishers.append(self.sonar_3_pub)

        self.sonar_4 = self.create_subscription(LaserScan, "/sonar_4_scan", self.sonar_callback,10 )
        self.sonar_4_pub = self.create_publisher(Range, "/sonar_4", 10)
        self.sonars_publishers.append(self.sonar_4_pub)
        
        self.sonar_5 = self.create_subscription(LaserScan, "/sonar_5_scan", self.sonar_callback,10 )
        self.sonar_5_pub = self.create_publisher(Range, "/sonar_5", 10)
        self.sonars_publishers.append(self.sonar_5_pub)


        self.sonar_6 = self.create_subscription(LaserScan, "/sonar_6_scan", self.sonar_callback,10 )
        self.sonar_6_pub = self.create_publisher(Range, "/sonar_6_", 10)
        self.sonars_publishers.append(self.sonar_6_pub)
        
        self.sonar_7 = self.create_subscription(LaserScan, "/sonar_7_scan", self.sonar_callback,10 )
        self.sonar_7_pub = self.create_publisher(Range, "/sonar_7", 10)
        self.sonars_publishers.append(self.sonar_7_pub)


        self.sonar_8 = self.create_subscription(LaserScan, "/sonar_8_scan", self.sonar_callback,10 )
        self.sonar_8_pub = self.create_publisher(Range, "/sonar_8", 10)
        self.sonars_publishers.append(self.sonar_8_pub)

        
        

        
        

    def wheels_callback(self, msg: WheelsData):


        left = Float64()
        left.data = (msg.left * 2 * math.pi)/60
        right = Float64()
        right.data = (msg.right * 2 * math.pi)/60
        self.left_wheel.publish(left)
        self.right_wheel.publish(right)

    def encoders_callback(self, msg: JointState):
        encoders = WheelsData()
        #self.get_logger().info(str(msg.velocity[0]) + " " + str(msg.velocity[1]))
        # TO_DO:check order in the array
        encoders.left = (float(msg.velocity[0]) * 60) / (2 * math.pi)
        encoders.right = (float(msg.velocity[1]) * 60)/ (2 * math.pi)
        self.encoders_pub.publish(encoders)

    def sonar_callback(self, msg: LaserScan):
        
        
        converted = Range()
        converted.header = msg.header
        converted.max_range = msg.range_max
        converted.min_range = msg.range_min
        converted.range = min([max([i,0]) for i in msg.ranges])
        converted.field_of_view = (msg.angle_max - msg.angle_min)
        self.sonars_publishers[int(str(msg.header.frame_id)[-1]) - 1].publish(converted)
        


def main(args=None):

    rclpy.init(args=args)

    converter = Converter()

    rclpy.spin(converter)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
