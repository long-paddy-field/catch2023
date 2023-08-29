#pragma once

#include <memory>
#include <thread>

#include "md_lib/odrive.hpp"
#include "principal_interfaces/action/commandpos.hpp"
#include "principal_interfaces/srv/commandvel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace catch2023_principal {

  
class Converter : public rclcpp::Node {
 public:
  // constructor
  Converter();
  // destructor
  ~Converter(){};

 private:
  // methods
  // 手動のときに使うcallback
  void commandvel_callback(
      const std::shared_ptr<principal_interfaces::srv::Commandvel::Request>
          request,
      std::shared_ptr<principal_interfaces::srv::Commandvel::Response>
          response);
  // 自動のときに使うcallback
  void commandpos_callback(
      const std::shared_ptr<principal_interfaces::action::Commandpos::Goal>
          goal);

  // member variables
  ODrive lower, middle;
  rclcpp_action::Server<principal_interfaces::action::Commandpos>::SharedPtr
      commandpos_server;
  rclcpp::Service<principal_interfaces::srv::Commandvel>::SharedPtr
      commandvel_service;
};
}  // namespace catch2023_principal