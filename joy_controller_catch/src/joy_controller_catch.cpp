#include <chrono>
#include <memory>

#include "catch2023_interfaces/msg/joy_command.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class JoyController : public rclcpp::Node {
 public:
  // constructor
  JoyController();

  // destructor
  ~JoyController() {}

 private:
  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // publisher
  rclcpp::Publisher<catch2023_interfaces::msg::JoyCommand>::SharedPtr joy_pub;

  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

  // message to publish
  catch2023_interfaces::msg::JoyCommand joy_cmd;

  // message to subscribe
  sensor_msgs::msg::Joy sub_msg;

  // callback
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  // main routine
  void update();

  // init_handle
  void init_handles();

  // calc messages
  catch2023_interfaces::msg::JoyCommand calc_msg();
};

JoyController::JoyController() : rclcpp::Node("joy_controller") {
  init_handles();
  get_params();
  timer_ =
      this->create_wall_timer(50ms, std::bind(&JoyController::update, this));
}

void JoyController::init_handles() {
  joy_pub = this->create_publisher<catch2023_interfaces::msg::JoyCommand>(
      "joy_cmd", 10);
  joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyController::joy_callback, this, _1));
}

void JoyController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  sub_msg = *msg;
}

catch2023_interfaces::msg::JoyCommand JoyController::calc_msg() {
  joy_cmd.arm_vel_c[0] = sub_msg.axes[0];
  joy_cmd.arm_vel_c[1] = sub_msg.axes[1];
  return joy_cmd;
}

void JoyController::update() { joy_pub->publish(calc_msg()); }

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}