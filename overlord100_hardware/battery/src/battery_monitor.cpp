#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include <chrono>
// #include <sys/types.h>
// #include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#define I2C_DEVICE_FILE "/dev/i2c-8"
#define ADS1115_ADDRESS 0x48
#define GAIN 0.001004292

using namespace std::chrono_literals;

class ADS1115 : public rclcpp::Node {
 public:
  ADS1115() : rclcpp::Node("battery_monitor") {
    timer = this->create_wall_timer(1s, std::bind(&ADS1115::call, this));
    pub = this->create_publisher<sensor_msgs::msg::BatteryState>(
        "battery_state", 10);
  }

 private:
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub;

  void call() {
    init();
    float voltage = measure();
    sensor_msgs::msg::BatteryState msg = sensor_msgs::msg::BatteryState();
    msg.voltage = voltage;
    msg.percentage = voltage2percentage(voltage);
    pub->publish(msg);
  }

  float voltage2percentage(float v) {
    float percentage = 1.23 * (1 - 1 / pow((1 + pow(v / 6 / 3.7, 80)), 0.165));
    return percentage > 1 ? 1 : percentage < 0.01 ? 0 : percentage;
  }

  int start_transmission() {
    int file;
    if ((file = open(I2C_DEVICE_FILE, O_RDWR)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Can't open i2c file");
      exit(1);
    }

    if (ioctl(file, I2C_SLAVE, ADS1115_ADDRESS) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Can't access i2c bus");
      exit(1);
    }
    return file;
  }

  void init() {
    int file = start_transmission();

    char config[3] = {(char)0x01, (char)0xC2, (char)0x83};
    write(file, config, 3);

    close(file);
  }

  float measure() {
    int file = start_transmission();

    char out_reg[1] = {0};
    write(file, out_reg, 1);

    char buf[2] = {0};
    float raw_data;
    float voltage;
    if (read(file, buf, 2) != 2) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read from the i2c bus");
    } else {
      raw_data = (float)((buf[0] << 8) | buf[1]);
      voltage = raw_data * GAIN;
      RCLCPP_INFO(this->get_logger(), "%f %f", raw_data, voltage);
    }

    close(file);

    return voltage;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ADS1115>());
  rclcpp::shutdown();
  return 0;
}
