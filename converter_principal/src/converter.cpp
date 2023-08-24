#include "converter.hpp"
using namespace catch2023_principal;

Converter::Converter() : Node("converter") {

}

Converter::~Converter(){

}


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Converter>());
  rclcpp::shutdown();
  return 0;
}