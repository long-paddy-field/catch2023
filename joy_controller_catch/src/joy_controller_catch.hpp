#pragma once

#include <chrono>
#include <math.hpp>
#include <memory>

#include "principal_interfaces/msg/joycommand.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace catch2023_principal {
enum class BUTTONS {
  X = 0,
  A = 1,
  B = 2,
  Y = 3,
  LB = 4,
  RB = 5,
  LT = 6,
  RT = 7,
  BACK = 8,
  START = 9,
  LS = 10,
  RS = 11,
};
class JoyControllerCatch : public rclcpp::Node {
 public:
  // constructor
  JoyControllerCatch();
  // destructor
  ~JoyControllerCatch(){};

 private:
  // member functions
  void update();

  // callback function
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void timer_callback();

  // member variables
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Publisher<principal_interfaces::msg::Joycommand>::SharedPtr
      joy_command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  principal_interfaces::msg::Joycommand joy_command_;
  bool is_connected_ = false;
  bool is_enabled_ = true;
  bool buttons_[12] = {false};
};

}  // namespace catch2023_principal
