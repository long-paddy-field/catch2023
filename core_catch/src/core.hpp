#pragma once

#include <chrono>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "principal_interfaces/action/commandpos.hpp"
#include "principal_interfaces/msg/joycommand.hpp"
#include "principal_interfaces/srv/commandvel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace catch2023_principal {
enum class Mode {
  MANUAL,
  AUTO,
  GUI,
};

typedef struct VELOCITY {
  float v_x;
  float v_y;
  float v_z;
  float v_yaw;
} VEL, *P_VEL;

typedef struct POSITION {
  float x;
  float y;
  float z;
  float yaw;
} POS, *P_POS;

class Core : public rclcpp::Node {
 public:
  // constructor
  Core();
  // destructor
  ~Core(){};

 private:
  // callbacks
  void update();
  void joyCallback(const principal_interfaces::msg::Joycommand::SharedPtr msg);

  // methodes
  void sendCommandVel(VEL cmd);
  void sendCommandPos(POS cmd);
  void sendTf();

  // member valuables
  POS current_pos;
  VEL command_vel;
  Mode mode;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<principal_interfaces::msg::Joycommand>::SharedPtr
      joy_subscriber;
  std::unique_ptr<tf2_ros::TransformBroadcaster> arm_tf;
  rclcpp::Client<principal_interfaces::srv::Commandvel>::SharedPtr
      command_vel_client;
};
}  // namespace catch2023_principal