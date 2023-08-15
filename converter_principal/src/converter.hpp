#pragma once

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "md_lib/odrive.hpp"

namespace catch2023_principal {
class Converter {
 public:
 private:
  // methods
  void update();
  void get_robot_state();
};
}  // namespace catch2023_principal