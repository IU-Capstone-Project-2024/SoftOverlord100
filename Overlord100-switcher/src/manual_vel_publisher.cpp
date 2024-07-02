
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
using namespace std::chrono_literals;

class ManualVelocityPublisher : public rclcpp::Node
{
public:
    ManualVelocityPublisher() : Node("manual_velocity_publisher")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("man_cmd_vel", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&ManualVelocityPublisher::publishVelocity, this));
    }

private:
    void publishVelocity()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.5;
        message.angular.z = 0.1;
        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%f, angular.z=%f", message.linear.x, message.angular.z);
        pub_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualVelocityPublisher>());
    rclcpp::shutdown();
}