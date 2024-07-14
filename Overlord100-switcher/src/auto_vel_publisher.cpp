#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class AutoVelocityPublisher : public rclcpp::Node {
  public:
    AutoVelocityPublisher() : Node("auto_velocity_publisher") {
        publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("auto_cmd_vel", 10);
        timer_ = this->create_wall_timer(
                     std::chrono::milliseconds(500),
                     std::bind(&AutoVelocityPublisher::publish_velocity, this));
    }

  private:
    void publish_velocity() {
        auto message = geometry_msgs::msg::Twist();
        message.linear.y = 5.5;   // Set desired linear velocity
        message.angular.z = 1.1;  // Set desired angular velocity
        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%f, angular.z=%f",
                    message.linear.x, message.angular.z);
        publisher_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoVelocityPublisher>());
    rclcpp::shutdown();
    return 0;
}