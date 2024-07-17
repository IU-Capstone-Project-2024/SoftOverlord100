// #include <cstdlib>
// #include <rclcpp/rclcpp.hpp>

// #include "nav2_msgs/srv/save_map.hpp"
// #include "overlord100_msgs/msg/map_name.hpp"
// #include "overlord100_msgs/srv/save_map_front_side.hpp"

// using std::placeholders::_1;

// class MapSaverNode : public rclcpp::Node {
//  public:
//   MapSaverNode() : Node("map_saver") {
//     map_saver_sub_ =
//     this->create_subscription<overlord100_msgs::msg::SaveMap>(
//         "map_saver_topic", 10,
//         std::bind(&MapSaverNode::mapSaverCallback, this,
//                   std::placeholders::_1));
//     map_name_sub_ =
//     this->create_subscription<overlord100_msgs::msg::MapName>(
//         "map_name", 10,
//         std::bind(&MapSaverNode::mapNameCallback, this,
//         std::placeholders::_1));
//     rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr map_saver_client =
//         this->create_client<nav2_msgs::srv::SaveMap>("map_saver");
//   }

//  private:
//   void changeModeCallback(
//       const std::shared_ptr<nav2_msgs::srv::SaveMap::Request> request,
//       std::shared_ptr<nav2_msgs::srv::SaveMap::Response> response) {
//     current_mode_ = request->mode;
//     if (current_mode_ == 1) {
//       std::string map_name_;
//       rclcpp::Subscription<overlord100_msgs::msg::SaveMap>::SharedPtr
//           map_saver_sub_;
//       rclcpp::Subscription<overlord100_msgs::msg::MapName>::SharedPtr
//           map_name_sub_;
//     };

//     int main(int argc, char* argv[]) {
//       rclcpp::init(argc, argv);
//       rclcpp::spin(std::make_shared<MapSaverNode>());
//       rclcpp::shutdown();
//       return 0;
//     }

// #include <memory>
// #include <string>
// // #include "nav2_msgs/srv/save_map.hpp"
// #include "overlord100_msgs/msg/save_map_front_side.hpp"
// #include "rclcpp/rclcpp.hpp"

// class MapSaverNode : public rclcpp::Node {
//  public:
//   MapSaverNode() : Node("map_saver_node") {
//     // Replace "map_saver_topic" with the actual topic name you are using
//     subscription_ =
//         this->create_subscription<overlord100_msgs::msg::SaveMapFrontSide>(
//             "map_saver_topic", 10,
//             std::bind(&MapSaverNode::topic_callback, this,
//                       std::placeholders::_1));
//   }

//  private:
//   void topic_callback(
//       const overlord100_msgs::msg::SaveMapFrontSide::SharedPtr msg) {
//     if (msg->save == 1) {
//       auto client =
//           this->create_client<nav2_msgs::srv::SaveMap>("/map_saver/save_map");

//       while (!client->wait_for_service(std::chrono::seconds(1))) {
//         if (!rclcpp::ok()) {
//           RCLCPP_ERROR(this->get_logger(),
//                        "Interrupted while waiting for the service.
//                        Exiting.");
//           return;
//         }
//         RCLCPP_INFO(this->get_logger(),
//                     "Service not available, waiting again...");
//       }

//       auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
//       // Replace "my_map" with the actual map name you want to use
//       request->map_topic = "map";
//       request->map_url = "my_map";
//       request->image_format = "pgm";
//       request->map_mode = "trinary";
//       request->free_thresh = 0.25;
//       request->occupied_thresh = 0.65;

//       auto result = client->async_send_request(request);

//       // Wait for the result.
//       if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
//                                              result) ==
//           rclcpp::FutureReturnCode::SUCCESS) {
//         RCLCPP_INFO(this->get_logger(), "Map saved successfully");
//       } else {
//         RCLCPP_ERROR(this->get_logger(), "Failed to call service save_map");
//       }
//     }
//   }

//   rclcpp::Subscription<overlord100_msgs::msg::SaveMapFrontSide>::SharedPtr
//       subscription_;
// };

// int main(int argc, char* argv[]) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MapSaverNode>());
//   rclcpp::shutdown();
//   return 0;
// }

#include "nav2_map_server/map_saver.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "overlord100_msgs/msg/save_map_front_side.hpp"
#include "rclcpp/rclcpp.hpp"

class CustomMapServerNode : public rclcpp::Node {
 public:
  CustomMapServerNode() : Node("custom_map_server_node") {
    // map_server_ =
    //     std::make_shared<nav2_map_server::MapServer>(shared_from_this());
    // map_saver_ =
    //     std::make_shared<nav2_map_server::MapSaver>(shared_from_this());

    map_saver_sub_ =
        this->create_subscription<overlord100_msgs::msg::SaveMapFrontSide>(
            "wheels_control", 10,
            std::bind(&CustomMapServerNode::topic_callback, this, _1));
  }
  // this->create_subscription<overlord100_msgs::msg::SaveMapFrontSide>(
  //     "/map_saver", rclcpp::QoS(10),
  //     std::bind(&CustomMapServerNode::mapSaverCallback, this,
  //               std::placeholders::_1));

  // map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
}

private : void
          mapSaverCallback(const std_msgs::msg::Int32::SharedPtr msg) {
  if (msg->data == 1) {
    RCLCPP_INFO(this->get_logger(), "Saving map...");

    auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
    request->map_topic = "map";
    request->map_url = "saved_map";
    request->image_format = "pgm";
    request->map_mode = "trinary";
    request->free_thresh = 0.25;
    request->occupied_thresh = 0.65;

    auto client =
        this->create_client<nav2_msgs::srv::SaveMap>("/map_saver/save_map");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Map saved successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call save map service");
    }
  }
}

// std::shared_ptr<nav2_map_server::MapServer> map_server_;
// std::shared_ptr<nav2_map_server::MapSaver> map_saver_;
rclcpp::Subscription<overlord100_msgs::msg::SaveMapFrontSide>::SharedPtr
    map_saver_sub_;
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
}
;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CustomMapServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
