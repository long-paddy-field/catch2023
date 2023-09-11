#include "converter.hpp"
using namespace catch2023_principal;
using namespace std::placeholders;

Converter::Converter()
    : Node("converter"),
      lower(this, "X", 0, 1.1, 1.8, 0.14, 3, is_auto),
      middle(this, "Y", -0.79, 0.79, 2.2, 0.21, 1.5, is_auto),
      arm(this, "arm") {
  // subscriberの初期設定
  manual_movecommand.x = 0;
  manual_movecommand.y = 0;
  manual_movecommand.z = 0;
  manual_movecommand.rotate = 0;
  manual_movecommand.hand[0] = false;
  manual_movecommand.hand[1] = false;
  manual_movecommand.hand[2] = false;
  auto_movecommand.x = 0;
  auto_movecommand.y = 0;
  auto_movecommand.z = 0;
  auto_movecommand.rotate = 0;
  auto_movecommand.hand[0] = false;
  auto_movecommand.hand[1] = false;
  auto_movecommand.hand[2] = false;
  RCLCPP_INFO(this->get_logger(), "start_init");
  lower.init_odrive();
  middle.init_odrive();
  middle.set_pos(0);
  arm.setZMode(Arm::ZMode::vel);
  manual_command_subscription =
      this->create_subscription<principal_interfaces::msg::Movecommand>(
          "manual_move_command", 10,
          std::bind(&Converter::manual_command_callback, this, _1));
  auto_command_subscription =
      this->create_subscription<principal_interfaces::msg::Movecommand>(
          "auto_move_command", 10,
          std::bind(&Converter::auto_command_callback, this, _1));
  is_auto_subscription = this->create_subscription<std_msgs::msg::Bool>(
      "is_auto", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (is_auto && !msg->data) {
          arm.setZMode(Arm::ZMode::vel);
          lower.change_mode_pos_to_vel();
          middle.change_mode_pos_to_vel();
        } else if (!is_auto && msg->data) {
          arm.setZMode(Arm::ZMode::pos);
          lower.change_mode_vel_to_pos();
          middle.change_mode_vel_to_pos();
        }
        is_auto = msg->data;
      });
  current_pos_publisher =
      this->create_publisher<principal_interfaces::msg::Movecommand>(
          "current_pos", 10);
  timer_ = this->create_wall_timer(100ms, std::bind(&Converter::update, this));
  RCLCPP_INFO(this->get_logger(), "end_init");
}

void Converter::manual_command_callback(
    const principal_interfaces::msg::Movecommand::SharedPtr msg) {
  if (!is_auto) {
    manual_movecommand.x = msg->x;
    manual_movecommand.y = msg->y;
    manual_movecommand.z = msg->z;
    manual_movecommand.rotate -= msg->rotate;
    if (manual_movecommand.rotate > 1) {
      manual_movecommand.rotate = 1;
    } else if (manual_movecommand.rotate < -1) {
      manual_movecommand.rotate = -1;
    }
    manual_movecommand.hand[0] =
        msg->hand[0] ? !manual_movecommand.hand[0] : manual_movecommand.hand[0];
    manual_movecommand.hand[1] =
        msg->hand[1] ? !manual_movecommand.hand[1] : manual_movecommand.hand[1];
    manual_movecommand.hand[2] =
        msg->hand[2] ? !manual_movecommand.hand[2] : manual_movecommand.hand[2];
  }
}

void Converter::auto_command_callback(
    const principal_interfaces::msg::Movecommand::SharedPtr msg) {
  if (is_auto) {
    auto_movecommand.x = msg->x;
    auto_movecommand.y = msg->y;
    auto_movecommand.z = msg->z;
    auto_movecommand.rotate = msg->rotate;
    auto_movecommand.hand[0] = msg->hand[0];
    auto_movecommand.hand[1] = msg->hand[1];
    auto_movecommand.hand[2] = msg->hand[2];
  }
}

void Converter::send_command() {
  if (is_auto) {
    lower.send_cmd_pos(auto_movecommand.x);
    middle.send_cmd_pos(auto_movecommand.y);
    arm.setZPos(auto_movecommand.z);
    arm.setArmAngle(static_cast<Arm::ArmAngle>(auto_movecommand.rotate));
    arm.setHand(auto_movecommand.hand[0], auto_movecommand.hand[1],
                auto_movecommand.hand[2]);
    manual_movecommand.hand[0] = auto_movecommand.hand[0];
    manual_movecommand.hand[1] = auto_movecommand.hand[1];
    manual_movecommand.hand[2] = auto_movecommand.hand[2];
  } else {
    lower.send_cmd_vel(manual_movecommand.x);
    middle.send_cmd_vel(manual_movecommand.y);
    arm.setZVel(manual_movecommand.z);
    arm.setArmAngle(static_cast<Arm::ArmAngle>(manual_movecommand.rotate));
    arm.setHand(manual_movecommand.hand[0], manual_movecommand.hand[1],
                manual_movecommand.hand[2]);
  }
}
void Converter::send_tf() {
  principal_interfaces::msg::Movecommand msg;
  msg.x = lower.get_pos();
  msg.y = middle.get_pos();
  msg.z = arm.getZPos();
  msg.rotate = is_auto ? auto_movecommand.rotate : manual_movecommand.rotate;
  msg.hand[0] = is_auto ? auto_movecommand.hand[0] : manual_movecommand.hand[0];
  msg.hand[1] = is_auto ? auto_movecommand.hand[1] : manual_movecommand.hand[1];
  msg.hand[2] = is_auto ? auto_movecommand.hand[2] : manual_movecommand.hand[2];
  current_pos_publisher->publish(msg);
}

void Converter::update() {
  send_command();
  send_tf();
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Converter>());
  rclcpp::shutdown();
  return 0;
}