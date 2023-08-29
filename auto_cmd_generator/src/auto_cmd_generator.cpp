#include "auto_cmd_generator.hpp"

using namespace catch2023_principal;

AutoCmdGenerator::AutoCmdGenerator() : Node("auto_cmd_generator") {
}



int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoCmdGenerator>());
  rclcpp::shutdown();
  return 0;
}