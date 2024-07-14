#include <chrono>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace std::chrono_literals;

class ManualVelocityPublisher : public rclcpp::Node {
  public:
    ManualVelocityPublisher() : Node("manual_velocity_publisher") {
        // Declare parameters
        this->declare_parameter<double>("linear_x", 0.0);
        this->declare_parameter<double>("angular_z", 0.0);
        this->declare_parameter<double>("linear_y", 0.0);

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("man_cmd_vel", 10);
        timer_ = this->create_wall_timer(
                     500ms, std::bind(&ManualVelocityPublisher::publishVelocity, this));
    }

  private:
    void publishVelocity() {
        // Get the parameter values
        double linear_x = this->get_parameter("linear_x").as_double();
        double linear_y = this->get_parameter("linear_y").as_double();
        double angular_z = this->get_parameter("angular_z").as_double();

        auto message = geometry_msgs::msg::Twist();
        message.linear.x = linear_x;
        message.linear.y = linear_y;
        message.angular.z = angular_z;
        message.angular.x = 0;
        message.angular.y = 0;
        message.linear.z = 0;

        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%f, angular.z=%f",
                    message.linear.x, message.angular.z);
        pub_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Create the node and set the parameters from the command line if provided
    auto node = std::make_shared<ManualVelocityPublisher>();

    // Get parameters from the command line arguments
    rclcpp::Parameter linear_x_param("linear_x", 0.0);
    rclcpp::Parameter angular_z_param("angular_z", 0.0);
    rclcpp::Parameter linear_y_param("linear_y", 0.0);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}