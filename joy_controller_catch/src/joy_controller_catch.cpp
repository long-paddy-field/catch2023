#include "joy_controller_catch.hpp"

using namespace catch2023_principal;
using namespace std::placeholders;
using namespace std::chrono_literals;

JoyControllerCatch::JoyControllerCatch() : Node("joy_controller_catch") {
  init_msg();
  init_btn();
  move_command_publisher_ =
      this->create_publisher<principal_interfaces::msg::Movecommand>(
          "manual_move_command", 10);
  state_command_publisher_ =
      this->create_publisher<principal_interfaces::msg::Statecommand>(
          "state_command", 10);
  is_auto_publisher_ =
      this->create_publisher<std_msgs::msg::Bool>("is_auto", 10);
  joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyControllerCatch::joy_callback, this, _1));
  param_subscriber =
      this->create_subscription<principal_interfaces::msg::Parameters>(
          "parameters", 10,
          std::bind(&JoyControllerCatch::config_params, this, _1));
  timer_ = this->create_wall_timer(
      50ms, std::bind(&JoyControllerCatch::timer_callback, this));
}
void JoyControllerCatch::init_msg() {
  move_command_.x = 0;
  move_command_.y = 0;
  move_command_.z = 0;
  move_command_.rotate = 0;
  state_command_.shift = 0;
  state_command_.phaze_change = 0;
  state_command_.is_common = 0;
  is_auto_.data = false;
  move_command_.hand[0] = false;
  move_command_.hand[1] = false;
  move_command_.hand[2] = false;
}
void JoyControllerCatch::init_btn() {
  buttons[static_cast<int>(BUTTONS::X)] = ButtonManager(BUTTON_TYPE::ON_OFF);
  buttons[static_cast<int>(BUTTONS::A)] = ButtonManager(BUTTON_TYPE::PULSER);
  buttons[static_cast<int>(BUTTONS::B)] = ButtonManager(BUTTON_TYPE::ON_OFF);
  buttons[static_cast<int>(BUTTONS::Y)] = ButtonManager(BUTTON_TYPE::ON_OFF);
  buttons[static_cast<int>(BUTTONS::START)] =
      ButtonManager(BUTTON_TYPE::PULSER);
  buttons[static_cast<int>(BUTTONS::BACK)] = ButtonManager(BUTTON_TYPE::PULSER);
  buttons[static_cast<int>(BUTTONS::LB)] = ButtonManager(BUTTON_TYPE::PULSER);
  buttons[static_cast<int>(BUTTONS::RB)] = ButtonManager(BUTTON_TYPE::PULSER);
  buttons[static_cast<int>(BUTTONS::LT)] =
      ButtonManager(BUTTON_TYPE::PUSH_RELEASE);
  buttons[static_cast<int>(BUTTONS::RT)] =
      ButtonManager(BUTTON_TYPE::PUSH_RELEASE);
  buttons[static_cast<int>(BUTTONS::LC)] = ButtonManager(BUTTON_TYPE::PULSER);
  buttons[static_cast<int>(BUTTONS::RC)] = ButtonManager(BUTTON_TYPE::PULSER);
  buttons[static_cast<int>(BUTTONS::LS)] = ButtonManager(BUTTON_TYPE::ON_OFF);
}
void JoyControllerCatch::config_params(
    const principal_interfaces::msg::Parameters::SharedPtr msg) {
  is_red = msg->isred;
  vel_max = msg->velmax;
  is_initialized = true;
}
void JoyControllerCatch::joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  is_connected = true;
  float slow = msg->buttons[static_cast<int>(BUTTONS::RT)] ? 0.25 : 1.0;
  move_command_.x = (is_red ? -1 : 1) * vel_max * msg->axes[0] * slow;
  move_command_.y = (is_red ? 1 : -1) * vel_max * msg->axes[1] * slow;
  move_command_.z = vel_max * msg->axes[3] * slow;

  buttons[static_cast<int>(BUTTONS::LC)].set(msg->axes[4] < -0.9);
  buttons[static_cast<int>(BUTTONS::RC)].set(msg->axes[4] > 0.9);
  for (int i = 4; i < 12; i++) {
    buttons[i].set(msg->buttons[i]);
  }
  buttons[static_cast<int>(BUTTONS::X)].set(
      msg->buttons[static_cast<int>(BUTTONS::A)] ||
      msg->buttons[static_cast<int>(BUTTONS::X)]);
  buttons[static_cast<int>(BUTTONS::Y)].set(
      msg->buttons[static_cast<int>(BUTTONS::A)] ||
      msg->buttons[static_cast<int>(BUTTONS::Y)]);
  buttons[static_cast<int>(BUTTONS::B)].set(
      msg->buttons[static_cast<int>(BUTTONS::A)] ||
      msg->buttons[static_cast<int>(BUTTONS::B)]);
}

void JoyControllerCatch::timer_callback() {
  RCLCPP_INFO(this->get_logger(), is_red ? "red" : "blue");
  if (is_connected && is_initialized) {
    move_command_.rotate = buttons[static_cast<int>(BUTTONS::LB)] -
                           buttons[static_cast<int>(BUTTONS::RB)];
    state_command_.shift = buttons[static_cast<int>(BUTTONS::LC)] -
                           buttons[static_cast<int>(BUTTONS::RC)];
    state_command_.phaze_change = buttons[static_cast<int>(BUTTONS::START)] -
                                  buttons[static_cast<int>(BUTTONS::BACK)];
    state_command_.is_common = buttons[static_cast<int>(BUTTONS::LS)];
    // RCLCPP_INFO(this->get_logger(),
    //             buttons[static_cast<int>(BUTTONS::UC)] ? "true" : "false");

    if (buttons[static_cast<int>(BUTTONS::A)]) {
    } else {
      move_command_.hand[0] = buttons[static_cast<int>(BUTTONS::X)];
      move_command_.hand[1] = buttons[static_cast<int>(BUTTONS::Y)];
      move_command_.hand[2] = buttons[static_cast<int>(BUTTONS::B)];
    }
    move_command_publisher_->publish(move_command_);
    state_command_publisher_->publish(state_command_);
    is_auto_publisher_->publish(is_auto_);
  }

  is_connected = false;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyControllerCatch>());
  rclcpp::shutdown();
  return 0;
}
