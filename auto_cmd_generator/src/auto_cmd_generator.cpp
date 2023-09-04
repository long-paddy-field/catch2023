#include "auto_cmd_generator.hpp"

#include <iostream>

using namespace catch2023_principal;

AutoCmdGenerator::AutoCmdGenerator() : Node("auto_cmd_generator") {
  timer = this->create_wall_timer(
      100ms, std::bind(&AutoCmdGenerator::timerCallback, this));
  state_command_subscription =
      this->create_subscription<principal_interfaces::msg::Statecommand>(
          "state_command", 10,
          [&](principal_interfaces::msg::Statecommand::SharedPtr stateCmd) {

          });
  is_auto_subscription = this->create_subscription<std_msgs::msg::Bool>(
      "is_auto", 10, [&, this](std_msgs::msg::Bool::SharedPtr isAuto) {});

  auto_command_publisher =
      this->create_publisher<principal_interfaces::msg::Movecommand>(
          "auto_move_command", 10);
  current_pos_subscription =
      this->create_subscription<principal_interfaces::msg::Movecommand>(
          "current_pos", 10,
          [&](principal_interfaces::msg::Movecommand::SharedPtr pos) {
            this->current_pos = pos;
          });

  this->declare_parameter("side", "blue");
  side = this->get_parameter("side").as_string() == "blue" ? Side::Blue
                                                           : Side::Red;

  this->declare_parameter("config_folder", std::string(__FILE__) + "../config");
  std::string config_folder = this->get_parameter("config_folder").as_string();

  transition(State::Init);
}

void AutoCmdGenerator::transition(State next_state) {
  switch (next_state) {
    case State::Init:
      break;
    case State::CatchMove:
      break;
    case State::CatchDown:
      break;
    case State::CatchHold:
      break;
    case State::CatchUp:
      break;
    case State::ShootDown:
      break;
    case State::ShootMove:
      break;
    case State::ShootRelease:
      break;
    case State::ShootUp:
      break;
    default:
      break;
  }
  state = next_state;
}

void AutoCmdGenerator::timerCallback() {
  switch (state) {
    case State::Init:

      break;
    case State::CatchMove:
      break;
    case State::CatchDown:
      break;
    case State::CatchHold:
      break;
    case State::CatchUp:
      break;
    case State::ShootDown:
      break;
    case State::ShootMove:
      break;
    case State::ShootRelease:
      break;
    case State::ShootUp:
      break;
    default:
      break;
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoCmdGenerator>());
  rclcpp::shutdown();
  return 0;
}