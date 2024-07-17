#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "overlord100_msgs/msg/wheels_data.hpp"
#include "can_msgs/msg/frame.hpp"
#include <algorithm>
#include <vector>

using std::placeholders::_1;
using namespace std::chrono_literals;

#define CAN_OUT_TOPIC "/CAN/can0/transmit"
#define CAN_IN_TOPIC "/CAN/can0/receive"

#define VELOCITY_CONTROL_TOPIC "/wheels_control"

#define NODE_ID 1

class MotorsDriverNode : public rclcpp::Node
{
  public:
    MotorsDriverNode()
    : Node("motors_driver_node")
    {
      // subscription_ = this->create_subscription<std_msgs::msg::String>(
      // "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

      wheels_velocities_input = this->create_subscription<overlord100_msgs::msg::WheelsData>(
      VELOCITY_CONTROL_TOPIC, 10, std::bind(&MotorsDriverNode::topic_callback, this, _1));

      can_output = this->create_publisher<can_msgs::msg::Frame>(CAN_OUT_TOPIC, 10);


      syncronous_velocity_control_init();

      // timer_ = this->create_wall_timer(500ms, std::bind(&MotorsDriverNode::syncronous_velocity_control_init, this));
    }

  private:
    void topic_callback(const overlord100_msgs::msg::WheelsData & msg)
    {
      //Make int
      int left = std::round(msg.left);
      int right = std::round(msg.right);

      //Bound velocity between -255 and 255 rev/min
      left = std::clamp(left, -0xFF, 0xFF);
      right = std::clamp(right, -0xFF, 0xFF);

      //Send data over CAN
      can_msgs::msg::Frame CANmsg = synchronized_velocity_send_frame((u_int16_t)left, (u_int16_t)right);
      can_output->publish(CANmsg);

    }

    can_msgs::msg::Frame synchronized_velocity_send_frame(u_int16_t l, u_int16_t r){
      // Server send function code = 1100 (0xC)
      u_int32_t COB_ID = (0xC << 7) + (u_int32_t)NODE_ID;

      std::array<uint8_t, 8UL> data;
      data[0] = 0x23; //Send request
      data[1] = 0xFF;
      data[2] = 0x60; //0x60FF - index
      data[3] = 0x03; //subindex
      data[4] = l;    //left wheel velocity 
      data[5] = l >> 8;
      data[6] = r;    //right wheel velocity 
      data[7] = r >> 8;

      can_msgs::msg::Frame frame;

      frame.id = COB_ID;
      frame.dlc = 8;
      frame.data = data;

      return frame;
    }

    void syncronous_velocity_control_init() {

      RCLCPP_INFO(this->get_logger(), "OK\n");

      std::vector<std::array<uint8_t, 8UL>> commands = std::vector<std::array<uint8_t, 8UL>>({
        {0x2B, 0x0F, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00},
        {0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00},
        {0x23, 0x83, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00},
        {0x23, 0x83, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00},
        {0x23, 0x84, 0x60, 0x01, 0x64, 0x00, 0x00, 0x00},
        {0x23, 0x84, 0x60, 0x02, 0x64, 0x00, 0x00, 0x00},
        {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00},
        {0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00},
        {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00},

      });

      // for(int i == 0; i < commands.size(); i++) {
      //   can_msgs::msg::Frame msg = can_msgs::msg::Frame();
      //   msg.
      // };

      for(std::array<uint8_t, 8UL> command : commands) {
        can_msgs::msg::Frame msg;
        msg.id = (0xC << 7) + (u_int32_t)NODE_ID;
        msg.dlc = 8;
        msg.data = command;

        can_output->publish(msg);
      }

    }


    rclcpp::Subscription<overlord100_msgs::msg::WheelsData>::SharedPtr wheels_velocities_input;

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_output;

    rclcpp::TimerBase::SharedPtr timer_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorsDriverNode>());
  rclcpp::shutdown();
  return 0;
}