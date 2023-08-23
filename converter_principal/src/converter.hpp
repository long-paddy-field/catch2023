#pragma once

#include <chrono>
#include <memory>

#include "md_lib/odrive.hpp"
#include "rclcpp/rclcpp.hpp"

namespace catch2023_principal {
class Converter : public rclcpp::Node {
 public:
  Converter();
  ~Converter();

 private:
  // methods
  void update();
  void get_robot_state();
};
}  // namespace catch2023_principal