import rclpy
from rclpy.node import Node
from overlord100_msgs.msg import WheelsData
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


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

    def wheels_callback(self, msg: WheelsData):
        self.left_wheel.publish(msg.left)
        self.right_wheel.publish(msg.right)

    def encoders_callback(self, msg: JointState):
        encoders = WheelsData()
        self.get_logger().info(str(msg.velocity[0]) + " " + str(msg.velocity[1]))
        # TO_DO:check order in the array
        encoders.left = float(msg.velocity[0])
        encoders.right = float(msg.velocity[1])
        self.encoders_pub.publish(encoders)


def main(args=None):

    rclpy.init(args=args)

    converter = Converter()

    rclpy.spin(converter)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
