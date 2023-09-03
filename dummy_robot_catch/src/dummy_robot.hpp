#pragma once

#include <chrono>
#include <memory>
#include <thread>

#include "principal_interfaces/msg/movecommand.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace catch2023_principal {
typedef struct MOVECOMMAND {
  float x;
  float y;
  float z;
  int rotate;
  bool hand[3];
} MOVECOMMAND;

class DummyRobot : public rclcpp::Node {
 public:
  // constructor
  DummyRobot();
  // destructor
  ~DummyRobot(){};

 private:
  // methods
  void update();
  // 手動のときに使うcallback
  void manual_command_callback(
      const principal_interfaces::msg::Movecommand::SharedPtr msg);
  // 自動のときに使うcallback
  void auto_command_callback(
      const principal_interfaces::msg::Movecommand::SharedPtr msg);
  // 指令値を送信
  void send_command();
  // tfに現在地を登録
  void send_tf();

  // member variables
  MOVECOMMAND movecommand;
  MOVECOMMAND current_pos;
  bool is_auto = false;

  rclcpp::Subscription<principal_interfaces::msg::Movecommand>::SharedPtr
      manual_command_subscription;
  rclcpp::Subscription<principal_interfaces::msg::Movecommand>::SharedPtr
      auto_command_subscription;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_auto_subscription;
  rclcpp::Publisher<principal_interfaces::msg::Movecommand>::SharedPtr
      current_pos_publisher;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace catch2023_principal