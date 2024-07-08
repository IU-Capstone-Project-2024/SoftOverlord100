
#include <rclcpp/rclcpp.hpp>
#include "overlord100_interfaces/srv/change_mode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <stdexcept>
#include <cstdio>
#include <memory>
#include <string>

class NavigationController : public rclcpp::Node
{
public:
    NavigationController() : Node("navigation_controller"), current_mode_(0)
    {

        mode_service_ = this->create_service<overlord100_interfaces::srv::ChangeMode>(
            "change_mode", std::bind(&NavigationController::changeModeCallback, this, std::placeholders::_1, std::placeholders::_2));

        manual_control_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "man_cmd_vel", 10, std::bind(&NavigationController::manualControlCallback, this, std::placeholders::_1));

        autonomous_control_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "auto_cmd_vel", 10, std::bind(&NavigationController::autonomousControlCallback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    void changeModeCallback(const std::shared_ptr<overlord100_interfaces::srv::ChangeMode::Request> request,
                            std::shared_ptr<overlord100_interfaces::srv::ChangeMode::Response> response)
    {
        current_mode_ = request->mode;
        if (current_mode_ == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Switching to autonomous mode");
            // Launch path planner using system call
            std::string command = "ros2 launch your_package path_planner.launch.py &";
            std::system(command.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Switching to manual mode");
        }
        response->success = true;
    }

    bool isVelocityValid(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        return (msg->angular.x == 0 && msg->angular.y == 0 && msg->linear.x < 100 &&
                msg->linear.y < 100 && msg->linear.z == 0);
    }

    void manualControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (current_mode_ == 0 && isVelocityValid(msg))
        {
            cmd_vel_pub_->publish(*msg);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Unreachable velocity");
        }
    }

    void autonomousControlCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (current_mode_ == 1)
        {
            cmd_vel_pub_->publish(*msg);
        }
    }

    int32_t current_mode_;
    rclcpp::Service<overlord100_interfaces::srv::ChangeMode>::SharedPtr mode_service_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_control_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr autonomous_control_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationController>());
    rclcpp::shutdown();
    return 0;
}