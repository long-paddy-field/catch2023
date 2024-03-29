#include "arm_lib/arm.hpp"

#include <cstring>
#include <rogilink2_interfaces/msg/detail/frame__struct.hpp>

Arm::Arm(rclcpp::Node *node, std::string deviceName)
    : node(node), deviceName(deviceName) {
  pub = node->create_publisher<rogilink2_interfaces::msg::Frame>(
      "rogilink2/send", 10);
  sub = node->create_subscription<rogilink2_interfaces::msg::Frame>(
      "rogilink2/receive_arm", 10, std::bind(&Arm::rogilinkCallback, this, _1));
  timer = node->create_wall_timer(100ms, std::bind(&Arm::timerCallback, this));
}

void Arm::setZMode(ZMode mode) { zMode = mode; }
void Arm::setZPos(float pos) { zPos = pos * STEP_PER_METER; }
void Arm::setZVel(float vel) { zVel = vel * STEP_PER_METER; }
void Arm::setArmAngle(ArmAngle angle) { armAngle = angle; }
void Arm::setHand(bool hand0, bool hand1, bool hand2) {
  handState[0] = hand0;
  handState[1] = hand1;
  handState[2] = hand2;
}

void Arm::rogilinkCallback(
    const rogilink2_interfaces::msg::Frame::SharedPtr msg) {
  if (msg->cmd_id != 0x03) return;
  currentZPos = *(float *)msg->data.data();
}

void Arm::timerCallback() {
  uint8_t data[8] = {0};
  *(float *)data = zMode == pos ? zPos : zVel;
  data[4] = zMode;
  data[5] = (char)armAngle;

  rogilink2_interfaces::msg::Frame frame;
  memcpy(frame.data.data(), data, 8);
  frame.name = deviceName;
  frame.cmd_id = 1;
  pub->publish(frame);

  for (int i = 0; i < 3; i++) {
    data[i] = handState[i];
  }
  rogilink2_interfaces::msg::Frame frame2;
  memcpy(frame2.data.data(), data, 8);
  frame2.name = deviceName + "_hand";
  frame2.cmd_id = 8;
  pub->publish(frame2);
}

float Arm::getZPos() { return currentZPos / STEP_PER_METER; }
