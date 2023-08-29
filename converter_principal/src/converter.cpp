#include "converter.hpp"
using namespace catch2023_principal;
using namespace std::placeholders;

Converter::Converter()
    : Node("converter"), lower(this, "X"), middle(this, "Y") {
  // ODriveの初期設定
  lower.init();
  middle.init();
  lower.setMode(Md::Mode::Velocity);
  middle.setMode(Md::Mode::Velocity);
  lower.setVelocity(0);
  middle.setVelocity(0);

  commandvel_service =
      this->create_service<principal_interfaces::srv::Commandvel>(
          "command_vel",
          std::bind(&Converter::commandvel_callback, this, _1, _2));
}

void Converter::commandvel_callback(
    const std::shared_ptr<principal_interfaces::srv::Commandvel::Request>
        request,
    std::shared_ptr<principal_interfaces::srv::Commandvel::Response> response){
  lower.setVelocity(request->x);
  middle.setVelocity(request->y);

  response->x = lower.getPosition();
  response->y = lower.getPosition();
}

    int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Converter>());
  rclcpp::shutdown();
  return 0;
}