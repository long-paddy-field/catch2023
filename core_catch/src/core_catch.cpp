#include "core_catch.hpp"
using namespace catch2023_principal;



int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoreCatch>());
  rclcpp::shutdown();
  return 0;
}