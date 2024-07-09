#include "rclcpp/rclcpp.hpp"
#include "overlord100_msgs/msg/log_message.hpp"
#include "rcl_interfaces/msg/log.hpp"

class LogCollectorNode : public rclcpp::Node
{
public:
    LogCollectorNode() : Node("log_collector")
    {
        log_publisher_ = this->create_publisher<overlord100_msgs::msg::LogMessage>("/logs", 10);
        log_subscription_ = this->create_subscription<rcl_interfaces::msg::Log>(
            "/rosout", 10, std::bind(&LogCollectorNode::logCallback, this, std::placeholders::_1));
    }

private:
    void logCallback(const rcl_interfaces::msg::Log::SharedPtr log_msg)
    {
        auto log = overlord100_msgs::msg::LogMessage();
        // log.header.stamp = this->now();
        log.node_name = log_msg->name;
        log.level = log_msg->level;
        log.message = log_msg->msg;

        log_publisher_->publish(log);
    }

    rclcpp::Publisher<overlord100_msgs::msg::LogMessage>::SharedPtr log_publisher_;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LogCollectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}