#include "rail_catch.hpp"

using namespace catch2023_principal;

RailCatch::RailCatch(rclcpp::Node *node, std::string name, float lower_limit,
                     float upper_limit, float vel_limit, float belt_ratio,
                     float arg)
    : node(node),
      odrive(node, name),
      lower_limit(lower_limit),
      upper_limit(upper_limit),
      vel_limit(vel_limit),
      belt_ratio(belt_ratio),
      arg(arg) {
  position = 0;
  is_auto = false;
  odrive.init();
  odrive.setMode(Md::Mode::Velocity);
  odrive.setVelocity(0);
}

void RailCatch::change_mode_pos_to_vel() {
  odrive.setVelocity(0);
  odrive.setPosition(odrive.getPosition());
  odrive.setMode(Md::Mode::Velocity,
                 ODriveEnum::InputMode::INPUT_MODE_PASSTHROUGH);
  odrive.setMode(Md::Mode::Position,
                 ODriveEnum::InputMode::INPUT_MODE_PASSTHROUGH);
  odrive.setMode(Md::Mode::Position);
  is_auto = false;
}

void RailCatch::change_mode_vel_to_pos() {
  odrive.setMode(Md::Mode::Position);
  is_auto = true;
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
      odrive.setVelocity(
          std::min(cmd / belt_ratio,
                   vel_limit * (1 - exp(arg * (position - upper_limit)))));

    } else {
      odrive.setVelocity(std::max(
          cmd / belt_ratio,
          vel_limit * (-1 + exp(-1 * arg * (position - lower_limit)))));
    }
  }
}

float RailCatch::get_pos() {
  position = odrive.getPosition() * belt_ratio;
  return position;
}