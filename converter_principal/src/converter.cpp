#include "converter.hpp"
using namespace catch2023_principal;
using namespace std::placeholders;

Converter::Converter()
    : Node("converter"),
      lower(this, "X", 0, 0, 0, 0, 0, is_auto),
      middle(this, "Y", 0, 0, 0, 0, 0, is_auto),
      arm(this, "arm") {
  // subscriberの初期設定
  manual_command_subscription =
      this->create_subscription<principal_interfaces::msg::Movecommand>(
          "manual_move_command", 10,
          std::bind(&Converter::manual_command_callback, this, _1));
  auto_command_subscription =
      this->create_subscription<principal_interfaces::msg::Movecommand>(
          "auto_move_command", 10,
          std::bind(&Converter::auto_command_callback, this, _1));
  is_auto_subscription = this->create_subscription<std_msgs::msg::Bool>(
      "is_auto", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (is_auto && !msg->data) {
          arm.setZMode(Arm::ZMode::vel);
        } else if (!is_auto && msg->data) {
          arm.setZMode(Arm::ZMode::pos);
        }
        is_auto = msg->data;
      });
  joint_state_publisher =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  timer_ = this->create_wall_timer(100ms, std::bind(&Converter::update, this));
}

void Converter::manual_command_callback(
    const principal_interfaces::msg::Movecommand::SharedPtr msg) {
  if (!is_auto) {
    movecommand.x = msg->x;
    movecommand.y = msg->y;
    movecommand.z = msg->z;
    movecommand.rotate = msg->rotate;
    movecommand.hand[0] = msg->hand[0];
    movecommand.hand[1] = msg->hand[1];
    movecommand.hand[2] = msg->hand[2];
  }
}

void Converter::auto_command_callback(
    const principal_interfaces::msg::Movecommand::SharedPtr msg) {
  if (is_auto) {
    movecommand.x = msg->x;
    movecommand.y = msg->y;
    movecommand.z = msg->z;
    movecommand.rotate = msg->rotate;
    movecommand.hand[0] = msg->hand[0];
    movecommand.hand[1] = msg->hand[1];
    movecommand.hand[2] = msg->hand[2];
  }
}

void Converter::send_command() {
  if (is_auto) {
    lower.send_cmd_pos(movecommand.y);
    middle.send_cmd_pos(movecommand.x);
    arm.setZPos(movecommand.z);
  } else {
    lower.send_cmd_vel(movecommand.y);
    middle.send_cmd_vel(movecommand.x);
    arm.setZVel(movecommand.z);
  }
  arm.setArmAngle(static_cast<Arm::ArmAngle>(movecommand.rotate));
  arm.setHand(movecommand.hand[0], movecommand.hand[1], movecommand.hand[2]);
  RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", movecommand.x,
              movecommand.y, movecommand.z);
}
void Converter::send_tf() {}

void Converter::update() {
  send_command();
  send_tf();
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Converter>());
  rclcpp::shutdown();
  return 0;
}