#include "joy_controller_catch.hpp"

using namespace catch2023_principal;
using namespace std::placeholders;
using namespace std::chrono_literals;

JoyControllerCatch::JoyControllerCatch() : Node("joy_controller_catch") {
  joy_command_publisher_ =
      this->create_publisher<principal_interfaces::msg::Joycommand>(
          "joy_command", 10);
  joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyControllerCatch::joy_callback, this, _1));
  timer_ = this->create_wall_timer(
      50ms, std::bind(&JoyControllerCatch::timer_callback, this));
  joy_command_.x = 0;
  joy_command_.y = 0;
  joy_command_.z = 0;
  joy_command_.rotate = 0;
  joy_command_.faze_change = 0;
  joy_command_.extend = false;
  joy_command_.slow = false;
  joy_command_.is_manual = true;
  joy_command_.hand[0] = false;
  joy_command_.hand[1] = false;
  joy_command_.hand[2] = false;
}

void JoyControllerCatch::joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  is_connected_ = true;
  joy_command_.x =
      -1 * msg->axes[1] / sqrt(pow(msg->axes[0], 2) + pow(msg->axes[1], 2));
  joy_command_.y =
      -1 * msg->axes[1] / sqrt(pow(msg->axes[0], 2) + pow(msg->axes[1], 2));
  joy_command_.z = -1 * msg->axes[3];
  for (int i = 0; i < 12; i++) {
    buttons_[i] = msg->buttons[i];
  }
}

void JoyControllerCatch::timer_callback() {
  if (is_connected_ && joy_command_.is_manual) {
    joy_command_.rotate =
        (joy_command_.rotate + buttons_[static_cast<int>(BUTTONS::X)] -
         buttons_[static_cast<int>(BUTTONS::B)]) %
            3 +
        1;
    joy_command_.faze_change = buttons_[static_cast<int>(BUTTONS::START)] -
                               buttons_[static_cast<int>(BUTTONS::BACK)];
    joy_command_.extend = buttons_[static_cast<int>(BUTTONS::LT)]
                              ? !joy_command_.extend
                              : joy_command_.extend;
    joy_command_.slow = buttons_[static_cast<int>(BUTTONS::RT)];
    joy_command_.is_manual = buttons_[static_cast<int>(BUTTONS::LS)]
                                 ? !joy_command_.is_manual
                                 : joy_command_.is_manual;
    joy_command_.hand[0] = buttons_[static_cast<int>(BUTTONS::Y)]
                               ? !joy_command_.hand[0]
                               : joy_command_.hand[0];
    joy_command_.hand[1] = buttons_[static_cast<int>(BUTTONS::A)]
                               ? !joy_command_.hand[1]
                               : joy_command_.hand[1];
    joy_command_.hand[2] = buttons_[static_cast<int>(BUTTONS::Y)]
                               ? !joy_command_.hand[2]
                               : joy_command_.hand[2];

    joy_command_publisher_->publish(joy_command_);
  }
  is_connected_ = false;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyControllerCatch>());
  rclcpp::shutdown();
  return 0;
}