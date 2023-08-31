#pragma once

#include <math.h>

#include <chrono>
#include <memory>

#include "principal_interfaces/msg/movecommand.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace catch2023_principal {
typedef struct CURRENTPOS {
  float x;
  float y;
  float z;
  float rotate;
} POS;

class Monitor : public rclcpp::Node {
 public:
  Monitor();
  ~Monitor(){};

 private:
  void current_pos_callback(
      const principal_interfaces::msg::Movecommand::SharedPtr msg);
  void update();

  POS current_pos;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  rclcpp::Subscription<principal_interfaces::msg::Movecommand>::SharedPtr
      current_pos_sub;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace catch2023_principal