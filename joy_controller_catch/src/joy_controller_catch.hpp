#pragma once

#include <math.h>

#include <chrono>
#include <memory>

#include "button_manager.hpp"
#include "principal_interfaces/msg/movecommand.hpp"
#include "principal_interfaces/msg/parameters.hpp"
#include "principal_interfaces/msg/statecommand.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

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
  // get params
  void config_params(
      const principal_interfaces::msg::Parameters::SharedPtr msg);
  // main routine
  void update();

  // callback function
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void timer_callback();

  // member variables
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Subscription<principal_interfaces::msg::Parameters>::SharedPtr
      param_subscriber;
  rclcpp::Publisher<principal_interfaces::msg::Movecommand>::SharedPtr
      move_command_publisher_;
  rclcpp::Publisher<principal_interfaces::msg::Statecommand>::SharedPtr
      state_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_auto_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  principal_interfaces::msg::Movecommand move_command_;
  principal_interfaces::msg::Statecommand state_command_;
  std_msgs::msg::Bool is_auto_;

  ButtonManager buttons[static_cast<int>(BUTTONS::BUTTON_NUM)];  // ボタン

  bool is_connected = false;
  bool is_initialized = false;
  bool is_common_buff;  // is_commonのやつ

  // パラメータ
  float vel_max = 0;  // 手動での速度最大値
  bool is_red = true;
};

}  // namespace catch2023_principal
