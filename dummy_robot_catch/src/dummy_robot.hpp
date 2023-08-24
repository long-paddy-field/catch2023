#pragma once

#include <chrono>
#include <memory>

#include "principal_interfaces/srv/commandvel.hpp"
#include "rclcpp/rclcpp.hpp"

namespace catch2023_principal {
class DummyRobot : public rclcpp::Node {
 public:
  // constructor
  DummyRobot();
  // destructor
  ~DummyRobot(){};

 private:
  // methods
    
  // callbacks
  void serviceCallback(
      const std::shared_ptr<principal_interfaces::srv::Commandvel::Request>
          request,
      std::shared_ptr<principal_interfaces::srv::Commandvel::Response>
          response);

  // member variables
  rclcpp::Service<principal_interfaces::srv::Commandvel>::SharedPtr service;

};
};  // namespace catch2023_principal