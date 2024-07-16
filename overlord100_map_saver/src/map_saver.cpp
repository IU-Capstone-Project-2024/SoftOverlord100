#include <cstdlib>
#include <rclcpp/rclcpp.hpp>

#include "nav2_msgs/srv/save_map.hpp"
#include "overlord100_msgs/msg/map_name.hpp"
#include "overlord100_msgs/msg/save_map.hpp"
using std::placeholders::_1;

class MapSaverNode : public rclcpp::Node {
 public:
  MapSaverNode() : Node("map_saver") {
    map_saver_sub_ = this->create_subscription<overlord100_msgs::msg::SaveMap>(
        "map_saver_topic", 10,
        std::bind(&MapSaverNode::mapSaverCallback, this,
                  std::placeholders::_1));
    map_name_sub_ = this->create_subscription<overlord100_msgs::msg::MapName>(
        "map_name", 10,
        std::bind(&MapSaverNode::mapNameCallback, this, std::placeholders::_1));
    rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr map_saver_client =
        node->create_client<nav2_msgs::srv::SaveMap>("map_saver");
  }

 private:
  void mapSaverCallback(const overlord100_msgs::msg::SaveMap::SharedPtr msg) {
    if (msg->save == 1) {
      saveMap();
    }
  }
  void mapNameCallback(const overlord100_msgs::msg::MapName::SharedPtr msg) {
    map_name_ = msg->name;
  }
  void saveMap() {
    RCLCPP_INFO(this->get_logger(), "Saving the map as %s", map_name_.c_str());

    // std::string command =
    //     "ros2 run nav2_map_server map_saver_cli -f " + map_name_;
    // std::system(command.c_str());
  }

  std::string map_name_;
  rclcpp::Subscription<overlord100_msgs::msg::SaveMap>::SharedPtr
      map_saver_sub_;
  rclcpp::Subscription<overlord100_msgs::msg::MapName>::SharedPtr map_name_sub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapSaverNode>());
  rclcpp::shutdown();
  return 0;
}
