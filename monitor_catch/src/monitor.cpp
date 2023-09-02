#include "monitor.hpp"

using namespace catch2023_principal;
using namespace std::placeholders;
using namespace std::chrono_literals;

Monitor::Monitor() : Node("monitor_catch") {
  current_pos.x = 0;
  current_pos.y = 0;
  current_pos.z = 0;
  current_pos.rotate = 0;

  joint_state_pub =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  current_pos_sub =
      this->create_subscription<principal_interfaces::msg::Movecommand>(
          "current_pos", 10,
          std::bind(&Monitor::current_pos_callback, this, _1));
  timer_ = this->create_wall_timer(100ms, std::bind(&Monitor::update, this));
}

void Monitor::current_pos_callback(
    const principal_interfaces::msg::Movecommand::SharedPtr msg) {
  current_pos.x = msg->x;
  current_pos.y = msg->y;
  current_pos.z = msg->z;
  current_pos.rotate = (msg->rotate + 1) * M_PI_2;
}

void Monitor::update() {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->get_clock()->now();
  msg.name.push_back("lower_platform");
  msg.position.push_back(current_pos.y);
  msg.name.push_back("platform_middle");
  msg.position.push_back(current_pos.x / 2.0);
  msg.name.push_back("middle_end");
  msg.position.push_back(current_pos.x / 2.0);

  msg.name.push_back("end_wrest");
  msg.position.push_back(current_pos.z);
  msg.name.push_back("wrest_hand");
  msg.position.push_back(current_pos.rotate);

  joint_state_pub->publish(msg);
  // RCLCPP_INFO(this->get_logger(), "Publishing: nya");
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Monitor>());
  rclcpp::shutdown();
  return 0;
}