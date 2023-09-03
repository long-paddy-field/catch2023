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
            float upper_limit, float vel_limit, float belt_ratio, float arg,
            bool &is_auto);
  ~RailCatch(){};

  void change_mode_vel_to_pos();
  void change_mode_pos_to_vel();
  void send_cmd_pos(float cmd);
  void send_cmd_vel(float cmd);
  void init_odrive();
  float get_pos();

 private:
  rclcpp::Node *node;
  ODrive odrive;
  // params
  float lower_limit;  // レール下限(m)
  float upper_limit;  // レール上限(m)

  float vel_limit;   // 速度上限
  float belt_ratio;  // モーター1回転で何m進むか
  float arg;         // 速度上限の重み
  bool is_auto;
  float past_cmd;  // 1tick前のcmd
  float position;
};
}  // namespace catch2023_principal