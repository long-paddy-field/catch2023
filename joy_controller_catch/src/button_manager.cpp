#include "button_manager.hpp"

using namespace catch2023_principal;

ButtonManager::ButtonManager(BUTTON_TYPE _button_type)
    : button_type(_button_type) {
  rclcpp::Time mytime = rclcpp::Clock().now();
  time_counter = mytime.nanoseconds();
}

void ButtonManager::set(bool btn) {
  rclcpp::Time mytime = rclcpp::Clock().now();
  if (mytime.nanoseconds() - time_counter > 0.2 * 1E9 && btn != past_btn) {
    on_off = !on_off;
    push_release = btn;
    time_counter = mytime.nanoseconds();
  }
  if (btn && !past_btn) {
    pulser = true;
  }
  past_btn = btn;
}

bool ButtonManager::read() {
  switch (button_type) {
    case BUTTON_TYPE::ON_OFF:
      return on_off;
      break;
    case BUTTON_TYPE::PUSH_RELEASE:
      return push_release;
      break;
    case BUTTON_TYPE::PULSER:
      if (pulser) {
        pulser = false;
        return true;
      } else {
        return false;
      }
    default:
      return false;
  }
}