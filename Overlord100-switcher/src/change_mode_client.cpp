#include <chrono>
#include <cstdlib>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "overlord100_interfaces/srv/change_mode.hpp"

using namespace std::chrono_literals;

class ModeChangeClient : public rclcpp::Node {
  public:
    ModeChangeClient() : Node("mode_change_client") {
        client_ = this->create_client<overlord100_interfaces::srv::ChangeMode>(
                      "change_mode");

        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(),
                             "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(),
                        "Service not available, waiting again...");
        }
    }

    bool send_request(int mode) {
        auto request =
            std::make_shared<overlord100_interfaces::srv::ChangeMode::Request>();
        request->mode = mode;

        auto result = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
                rclcpp::FutureReturnCode::SUCCESS) {
            return result.get()->success;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service change_mode");
            return false;
        }
    }

  private:
    rclcpp::Client<overlord100_interfaces::srv::ChangeMode>::SharedPtr client_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc != 2) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Usage: change_mode_client <mode>");
        return 1;
    }

    int mode = std::atoi(argv[1]);

    auto node = std::make_shared<ModeChangeClient>();
    bool success = node->send_request(mode);

    if (success) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mode change successful");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Mode change failed");
    }

    rclcpp::shutdown();
    return 0;
}