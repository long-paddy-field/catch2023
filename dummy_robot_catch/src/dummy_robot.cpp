#include "dummy_robot.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace catch2023_principal;

DummyRobot::DummyRobot() : Node("dummy_robot") {
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
  timer_ = this->create_wall_timer(100ms, std::bind(&update, this));
}

void DummyRobot::manual_command_callback(
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

void DummyRobot::send_command(){

}

void DummyRobot::send_tf(){
  
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyRobot>());
  rclcpp::shutdown();
  return 0;
}