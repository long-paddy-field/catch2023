#include "converter.hpp"
using namespace catch2023_principal;
using namespace std::placeholders;

Converter::Converter()
    : Node("converter"),
      lower(this, "X", 0, 0, 0, 0, 0),
      middle(this, "Y", 0, 0, 0, 0, 0) {
  // ODriveの初期設定

  commandvel_service =
      this->create_service<principal_interfaces::srv::Commandvel>(
          "command_vel",
          std::bind(&Converter::commandvel_callback, this, _1, _2));
}

void Converter::commandvel_callback(
    const std::shared_ptr<principal_interfaces::srv::Commandvel::Request>
        request,
    std::shared_ptr<principal_interfaces::srv::Commandvel::Response> response) {
  lower.send_cmd_vel(request->x);
  middle.send_cmd_vel(request->y);

  response->x = lower.get_pos();
  response->y = middle.get_pos();
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Converter>());
  rclcpp::shutdown();
  return 0;
}