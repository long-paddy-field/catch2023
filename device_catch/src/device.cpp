#include "rclcpp/rclcpp.hpp"

class Device : public rclcpp::Node {
 public:
  Device() : Node("device_catch") {}

 private:
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Device>());
  rclcpp::shutdown();
  return 0;
}