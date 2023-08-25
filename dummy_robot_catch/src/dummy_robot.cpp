#include "dummy_robot.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace catch2023_principal;

DummyRobot::DummyRobot() : Node("dummy_robot") {
  service = this->create_service<principal_interfaces::srv::Commandvel>(
      "command_vel", std::bind(&DummyRobot::serviceCallback, this, _1, _2));
  RCLCPP_INFO(this->get_logger(), "Dummy robot initialized");
}

void DummyRobot::serviceCallback(
    const std::shared_ptr<principal_interfaces::srv::Commandvel::Request>
        request,
    std::shared_ptr<principal_interfaces::srv::Commandvel::Response> response) {
  response->x = request->x;
  response->y = request->y;
  response->z = request->z;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyRobot>());
  rclcpp::shutdown();
  return 0;
}