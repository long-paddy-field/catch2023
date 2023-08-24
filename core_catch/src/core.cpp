#include "core.hpp"
using namespace std::chrono_literals;
using namespace catch2023_principal;

Core::Core() : Node("core") {
  arm_tf = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  command_vel_client =
      this->create_client<principal_interfaces::srv::Commandvel>("command_vel");
  joy_subscriber =
      this->create_subscription<principal_interfaces::msg::Joycommand>(
          "joy_command", 10,
          std::bind(&Core::joyCallback, this, std::placeholders::_1));
  current_pos.x = 0;
  current_pos.y = 0;
  current_pos.z = 0;
  current_pos.yaw = 0;
  RCLCPP_INFO(this->get_logger(), "Core initialized");
  update();
}

void Core::update() {
  RCLCPP_INFO(this->get_logger(), "Update");
  while (rclcpp::ok()) {
    switch (mode) {
      case Mode::MANUAL:
        sendCommandVel(command_vel);
        RCLCPP_INFO(this->get_logger(), "Manual mode");
        break;
      case Mode::AUTO:
        break;
      case Mode::GUI:
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Invalid mode");
    }
    RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", current_pos.x,
                current_pos.y, current_pos.z);
    // sendTf();
  }
}

void Core::joyCallback(
    const principal_interfaces::msg::Joycommand::SharedPtr msg) {
  command_vel.v_x = msg->x;
  command_vel.v_y = msg->y;
  command_vel.v_z = msg->z;
  mode = msg->is_auto ? Mode::AUTO : Mode::MANUAL;
}

void Core::sendCommandVel(VEL cmd) {
  auto request =
      std::make_shared<principal_interfaces::srv::Commandvel::Request>();
  request->x = cmd.v_x;
  request->y = cmd.v_y;
  request->z = cmd.v_z;
  RCLCPP_INFO(this->get_logger(), "Sending request");
  while (!command_vel_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }
  // サービスのレスポンスをcurrent_posに代入
  auto result = command_vel_client->async_send_request(
      request,
      std::bind(&Core::commandvelCallback, this, std::placeholders::_1));
}

void Core::commandvelCallback(
    const rclcpp::Client<principal_interfaces::srv::Commandvel>::SharedFuture
        response) {
  RCLCPP_INFO(this->get_logger(), "Callback");
  current_pos.x = response.get()->x;
  current_pos.y = response.get()->y;
  current_pos.z = response.get()->z;
  RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", current_pos.x,
              current_pos.y, current_pos.z);
}

void Core::sendTf() {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "arm_link";

  t.transform.translation.x = current_pos.x;
  t.transform.translation.y = current_pos.y;
  t.transform.translation.z = current_pos.z;

  tf2::Quaternion q;
  q.setRPY(0, 0, current_pos.yaw);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  arm_tf->sendTransform(t);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Core>());
  rclcpp::shutdown();
  return 0;
}