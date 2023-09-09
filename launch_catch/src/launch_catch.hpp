#pragma once

#include <chrono>
#include <vector>

#include "principal_interfaces/msg/parameters.hpp"
#include "rclcpp/rclcpp.hpp"
namespace catch2023_principal {
class ParameterManager : public rclcpp::Node {
 public:
  ParameterManager();
  ~ParameterManager();

 private:
  // methods
  void load_param();  // load param from yaml
  void save_param();  // save param to yaml
  void get_state(
      principal_interfaces::msg::Parameters::SharedPtr);  // get state from
                                                          // param_manager

  // handles
  rclcpp::Publisher<principal_interfaces::msg::Parameters>::SharedPtr
      param_pub_;  // publish param to param_manager
//   rclcpp::Subscription<principal_interfaces::msg::Parameters>::SharedPtr
//       param_sub_;  // subscribe state from param_manager
};
}  // namespace catch2023_principal