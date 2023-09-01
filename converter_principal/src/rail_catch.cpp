#include "rail_catch.hpp"

using namespace catch2023_principal;

RailCatch::RailCatch(rclcpp::Node *node, std::string name, float lower_limit,
                     float upper_limit, float vel_limit, float belt_ratio,
                     float arg, bool &is_auto)
    : node(node),
      odrive(node, name),
      lower_limit(lower_limit),
      upper_limit(upper_limit),
      vel_limit(vel_limit),
      belt_ratio(belt_ratio),
      arg(arg),
      is_auto(is_auto) {
  RCLCPP_INFO(node->get_logger(), "rail_init_start");
  position = 0;
  past_cmd = 0;
}
void RailCatch::init_odrive() {
  odrive.init();
  odrive.setMode(Md::Mode::Position,
                 ODriveEnum::InputMode::INPUT_MODE_TRAP_TRAJ);
  odrive.setVelocity(0);
}
void RailCatch::send_cmd_pos(float cmd) {
  if (is_auto) {
    odrive.setPosition(cmd / belt_ratio);
  } else {
    RCLCPP_ERROR(node->get_logger(), "This is not available in manual mode");
  }
}

void RailCatch::send_cmd_vel(float cmd) {
  if (is_auto) {
    RCLCPP_ERROR(node->get_logger(), "This is not available in auto mode");
  } else {
    // 速度上限を超えない範囲でsetVel,速度上限は限界に近いほど小さくなる
    if (cmd > 0) {
      if (past_cmd <= 0) {
        odrive.setPosition(odrive.getPosition());
        odrive.setMode(Md::Mode::Position,
                       ODriveEnum::InputMode::INPUT_MODE_PASSTHROUGH);
        odrive.setMode(Md::Mode::Position,
                       ODriveEnum::InputMode::INPUT_MODE_TRAP_TRAJ);
      }
      odrive.setPosition(0);
      odrive.setLimits(cmd / belt_ratio, 30);
    } else {
      if (past_cmd >= 0) {
        odrive.setPosition(odrive.getPosition());
        odrive.setMode(Md::Mode::Position,
                       ODriveEnum::InputMode::INPUT_MODE_PASSTHROUGH);
        odrive.setMode(Md::Mode::Position,
                       ODriveEnum::InputMode::INPUT_MODE_TRAP_TRAJ);
      }
      odrive.setPosition((upper_limit - lower_limit) / belt_ratio);
      odrive.setLimits(abs(cmd) / belt_ratio, 30);
    }
  }
  past_cmd = cmd;
}

float RailCatch::get_pos() {
  position = odrive.getPosition() * belt_ratio;
  return position;
}