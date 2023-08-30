#pragma once

#include <math.h>

#include <memory>
#include <string>

#include "md_lib/odrive.hpp"
#include "rclcpp/rclcpp.hpp"

namespace catch2023_principal {
class RailCatch {
 public:
  RailCatch(rclcpp::Node *node, std::string name, float lower_limit,
            float upper_limit, float vel_limit, float ratio, float arg);
  ~RailCatch(){};
  void change_mode_vel_to_pos();
  void change_mode_pos_to_vel();
  void send_cmd_pos(float cmd);
  void send_cmd_vel(float cmd);
  float get_pos();

 private:
  ODrive odrive;
  rclcpp::Node *node;
  // params
  float belt_ratio;   // モーター1回転で何m進むか
  float upper_limit;  // レール上限(m)
  float lower_limit;  // レール下限(m)
  float vel_limit;    // 速度上限
  float arg;          // 速度上限の重み

  float position;
  bool is_auto;
};
}  // namespace catch2023_principal