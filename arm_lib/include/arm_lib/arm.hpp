#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rogilink2_interfaces/msg/frame.hpp"
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <string>

using namespace std::placeholders;
using namespace std::chrono_literals;

class Arm {
public:
  Arm(rclcpp::Node *node, std::string deviceName);

  enum ZMode { vel = 0, pos = 1 };
  enum ArmAngle { ownArea, commonArea };

  void setZMode(ZMode mode);
  void setZPos(float pos);
  void setZVel(float vel);

  void setArmAngle(ArmAngle angle);

  void setHand(bool hand0, bool hand1, bool hand2);

  float getZPos();

private:
  const rclcpp::Node *node;
  const std::string deviceName;
  rclcpp::Publisher<rogilink2_interfaces::msg::Frame>::SharedPtr pub;
  rclcpp::Subscription<rogilink2_interfaces::msg::Frame>::SharedPtr sub;
  rclcpp::TimerBase::SharedPtr timer;

  void rogilinkCallback(const rogilink2_interfaces::msg::Frame::SharedPtr msg);
  void timerCallback();

  ZMode zMode = pos;
  ArmAngle armAngle = ownArea;
  bool handState[3] = {0};

  float zPos = 0;
  float zVel = 0;

  float currentZPos = 0;
};