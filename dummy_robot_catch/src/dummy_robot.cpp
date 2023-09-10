#include "dummy_robot.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace catch2023_principal;

DummyRobot::DummyRobot() : Node("dummy_robot") {
  movecommand.x = 0;
  movecommand.y = 0;
  movecommand.z = 0;
  movecommand.rotate = 0;
  movecommand.hand[0] = false;
  movecommand.hand[1] = false;
  movecommand.hand[2] = false;

  manual_command_subscription =
      this->create_subscription<principal_interfaces::msg::Movecommand>(
          "manual_move_command", 10,
          std::bind(&DummyRobot::manual_command_callback, this, _1));
  auto_command_subscription =
      this->create_subscription<principal_interfaces::msg::Movecommand>(
          "auto_move_command", 10,
          std::bind(&DummyRobot::auto_command_callback, this, _1));
  is_auto_subscription = this->create_subscription<std_msgs::msg::Bool>(
      "is_auto", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        is_auto = msg->data;
      });
  current_pos_publisher =
      this->create_publisher<principal_interfaces::msg::Movecommand>(
          "current_pos", 10);
  timer_ = this->create_wall_timer(10ms, std::bind(&DummyRobot::update, this));
}

void DummyRobot::manual_command_callback(
    const principal_interfaces::msg::Movecommand::SharedPtr msg) {
  if (!is_auto) {
    movecommand.x = msg->x;
    movecommand.y = msg->y;
    movecommand.z = msg->z;
    movecommand.rotate = msg->rotate;
    movecommand.hand[0] =
        msg->hand[0] ? !movecommand.hand[0] : movecommand.hand[0];
    movecommand.hand[1] =
        msg->hand[1] ? !movecommand.hand[1] : movecommand.hand[1];
    movecommand.hand[2] =
        msg->hand[2] ? !movecommand.hand[2] : movecommand.hand[2];
  }
}

void DummyRobot::auto_command_callback(
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

void DummyRobot::update() {
  send_command();
  send_tf();
}

void DummyRobot::send_command() {
  if (is_auto) {
    current_pos.x += (movecommand.x - current_pos.x) / 10;
    current_pos.y += (movecommand.y - current_pos.y) / 10;
    current_pos.z += (movecommand.z - current_pos.z) / 10;
    current_pos.rotate = movecommand.rotate;
    current_pos.hand[0] = movecommand.hand[0];
    current_pos.hand[1] = movecommand.hand[1];
    current_pos.hand[2] = movecommand.hand[2];
  } else {
    current_pos.x += movecommand.x / 100;
    current_pos.y += movecommand.y / 100;
    current_pos.z += movecommand.z / 100;
    current_pos.rotate += movecommand.rotate;
    current_pos.hand[0] += movecommand.hand[0];
    current_pos.hand[1] += movecommand.hand[1];
    current_pos.hand[2] += movecommand.hand[2];
  }
}

void DummyRobot::send_tf() {
  principal_interfaces::msg::Movecommand msg;
  msg.x = current_pos.x;
  msg.y = current_pos.y;
  msg.z = current_pos.z;
  msg.rotate = current_pos.rotate;
  msg.hand[0] = current_pos.hand[0];
  msg.hand[1] = current_pos.hand[1];
  msg.hand[2] = current_pos.hand[2];

  current_pos_publisher->publish(msg);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyRobot>());
  rclcpp::shutdown();
  return 0;
}