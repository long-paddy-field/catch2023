#pragma once

#include <math.h>

#include <chrono>
#include <memory>

#include "button_manager.hpp"
#include "principal_interfaces/msg/joycommand.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace catch2023_principal {

class JoyControllerCatch : public rclcpp::Node {
 public:
  // constructor
  JoyControllerCatch();
  // destructor
  ~JoyControllerCatch(){};

 private:
  // member functions

  // initialize msg
  void init_msg();
  // initialize button class
  void init_btn();
  // main routine
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
  ButtonManager buttons[14];
  bool is_connected = false;
};

}  // namespace catch2023_principal
