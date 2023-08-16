#pragma once

#include "rclcpp/rclcpp.hpp"

class Arm {
public:
  Arm(rclcpp::Node *node, uint8_t hard_id);

private:
  const rclcpp::Node *node;
  const uint8_t hard_id;
};

