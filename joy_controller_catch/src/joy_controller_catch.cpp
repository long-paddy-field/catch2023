#include "joy_controller_catch.hpp"

using namespace catch2023_principal;
using namespace std::placeholders;
using namespace std::chrono_literals;

JoyControllerCatch::JoyControllerCatch() : Node("joy_controller_catch") {
  init_msg();
  init_btn();
  joy_command_publisher_ =
      this->create_publisher<principal_interfaces::msg::Joycommand>(
          "joy_command", 10);
  joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyControllerCatch::joy_callback, this, _1));
  timer_ = this->create_wall_timer(
      50ms, std::bind(&JoyControllerCatch::timer_callback, this));
  vel_max = 0.5;
}
void JoyControllerCatch::init_msg() {
  joy_command_.x = 0;
  joy_command_.y = 0;
  joy_command_.z = 0;
  joy_command_.extend = 0;
  joy_command_.rotate = 0;
  joy_command_.shift = 0;
  joy_command_.faze_change = 0;
  joy_command_.slow = false;
  joy_command_.is_auto = false;
  joy_command_.hand[0] = false;
  joy_command_.hand[1] = false;
  joy_command_.hand[2] = false;
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
}
void JoyControllerCatch::joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  is_connected = true;
  joy_command_.x = vel_max * msg->axes[1];
  joy_command_.y = vel_max * msg->axes[0];
  joy_command_.z = vel_max * msg->axes[3];
  joy_command_.extend = msg->axes[2];

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
  if (is_connected) {
    joy_command_.rotate = buttons[static_cast<int>(BUTTONS::LB)] -
                          buttons[static_cast<int>(BUTTONS::RB)];
    joy_command_.shift = buttons[static_cast<int>(BUTTONS::LC)] -
                         buttons[static_cast<int>(BUTTONS::RC)];
    joy_command_.faze_change = buttons[static_cast<int>(BUTTONS::START)] -
                               buttons[static_cast<int>(BUTTONS::BACK)];
    joy_command_.is_auto = buttons[static_cast<int>(BUTTONS::LT)];
    joy_command_.slow = buttons[static_cast<int>(BUTTONS::RT)];
    if (buttons[static_cast<int>(BUTTONS::A)]) {
    } else {
      joy_command_.hand[0] = buttons[static_cast<int>(BUTTONS::X)];
      joy_command_.hand[1] = buttons[static_cast<int>(BUTTONS::Y)];
      joy_command_.hand[2] = buttons[static_cast<int>(BUTTONS::B)];
    }
    joy_command_publisher_->publish(joy_command_);
  }

  is_connected = false;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyControllerCatch>());
  rclcpp::shutdown();
  return 0;
}