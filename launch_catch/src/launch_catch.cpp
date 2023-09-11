#include "launch_catch.hpp"

using namespace catch2023_principal;
using namespace std::chrono_literals;

ParameterManager::ParameterManager() : Node("parameter_manager") {
  param_pub_ = this->create_publisher<principal_interfaces::msg::Parameters>(
      "parameters", 10);
  // param_sub_ =
  // this->create_subscription<principal_interfaces::msg::Parameters>(
  //     "current_state", 10,
  //     std::bind(&ParameterManager::get_state, this, std::placeholders::_1));
  load_param();
  timer_ = this->create_wall_timer(
      50ms, std::bind(&ParameterManager::send_param, this));
}

ParameterManager::~ParameterManager() { save_param(); }

void ParameterManager::load_param() {
  // declare_parameters
  this->declare_parameter("field_color", "red");
  this->declare_parameter("max_vel", 1.0);
  this->declare_parameter("arm_offset", 1.0);
  this->declare_parameter("cmn_offset", 1.0);
  this->declare_parameter("sht_offset", 1.0);
  this->declare_parameter("hand_offset", 1.0);
  this->declare_parameter("stepper_state", std::vector<double>(9, 0.0));
  msg.isred = (this->get_parameter("field_color").as_string() == "red");
  RCLCPP_INFO(this->get_logger(), "field_color: %s",
              msg.isred ? "red" : "blue");
  msg.velmax = (float)this->get_parameter("max_vel").as_double();
  msg.armoffset = (float)this->get_parameter("arm_offset").as_double();
  msg.cmnoffset = (float)this->get_parameter("cmn_offset").as_double();
  msg.shtoffset = (float)this->get_parameter("sht_offset").as_double();
  msg.handoffset = (float)this->get_parameter("hand_offset").as_double();
  if (msg.isred) {
    this->declare_parameter("red_side.start_pos", std::vector<double>(2, 0.0));
    this->declare_parameter("red_side.way_point", std::vector<double>(2, 0.0));
    this->declare_parameter("red_side.own_pos_x", std::vector<double>(16, 0.0));
    this->declare_parameter("red_side.own_pos_y", std::vector<double>(16, 0.0));
    this->declare_parameter("red_side.cmn_pos_x", std::vector<double>(9, 0.0));
    this->declare_parameter("red_side.cmn_pos_y", std::vector<double>(9, 0.0));
    this->declare_parameter("red_side.sht_pos_x", std::vector<double>(9, 0.0));
    this->declare_parameter("red_side.sht_pos_y", std::vector<double>(9, 0.0));
    this->declare_parameter("red_side.own_field", std::vector<double>(4, 0.0));
    this->declare_parameter("red_side.cmn_field", std::vector<double>(4, 0.0));
    this->declare_parameter("red_side.sht_field", std::vector<double>(4, 0.0));

    std::vector<double> buff =
        this->get_parameter("red_side.start_pos").as_double_array();
    msg.startpos[0] = (float)buff[0];
    msg.startpos[1] = (float)buff[1];
    buff.clear();
    buff = this->get_parameter("red_side.way_point").as_double_array();
    msg.waypoint[0] = (float)buff[0];
    msg.waypoint[1] = (float)buff[1];
    buff.clear();
    buff = this->get_parameter("red_side.own_pos_x").as_double_array();
    for (int i = 0; i < 16; i++) {
      msg.ownx[i] = buff[i];
    }
    buff.clear();
    buff = this->get_parameter("red_side.own_pos_y").as_double_array();
    for (int i = 0; i < 16; i++) {
      msg.owny[i] = buff[i];
    }
    buff.clear();
    buff = this->get_parameter("red_side.cmn_pos_x").as_double_array();
    for (int i = 0; i < 9; i++) {
      msg.cmnx[i] = buff[i];
    }
    buff.clear();
    buff = this->get_parameter("red_side.cmn_pos_y").as_double_array();
    for (int i = 0; i < 9; i++) {
      msg.cmny[i] = buff[i];
    }
    buff.clear();
    buff = this->get_parameter("red_side.sht_pos_x").as_double_array();
    for (int i = 0; i < 9; i++) {
      msg.shtx[i] = buff[i];
    }
    buff.clear();
    buff = this->get_parameter("red_side.sht_pos_y").as_double_array();
    for (int i = 0; i < 9; i++) {
      msg.shty[i] = buff[i];
    }
    buff.clear();
    buff = this->get_parameter("red_side.own_field").as_double_array();
    for (int i = 0; i < 4; i++) {
      msg.ownfield[i] = buff[i];
    }
    buff.clear();
    buff = this->get_parameter("red_side.cmn_field").as_double_array();
    for (int i = 0; i < 4; i++) {
      msg.cmnfield[i] = buff[i];
    }
    buff.clear();
    buff = this->get_parameter("red_side.sht_field").as_double_array();
    for (int i = 0; i < 4; i++) {
      msg.shtfield[i] = buff[i];
    }
    buff.clear();
  } else {
    this->declare_parameter("blue_side.start_pos", std::vector<double>(2, 0.0));
    this->declare_parameter("blue_side.way_point", std::vector<double>(2, 0.0));
    this->declare_parameter("blue_side.own_pos_x",
                            std::vector<double>(16, 0.0));
    this->declare_parameter("blue_side.own_pos_y",
                            std::vector<double>(16, 0.0));
    this->declare_parameter("blue_side.cmn_pos_x", std::vector<double>(9, 0.0));
    this->declare_parameter("blue_side.cmn_pos_y", std::vector<double>(9, 0.0));
    this->declare_parameter("blue_side.sht_pos_x", std::vector<double>(9, 0.0));
    this->declare_parameter("blue_side.sht_pos_y", std::vector<double>(9, 0.0));
    this->declare_parameter("blue_side.own_field", std::vector<double>(4, 0.0));
    this->declare_parameter("blue_side.cmn_field", std::vector<double>(4, 0.0));
    this->declare_parameter("blue_side.sht_field", std::vector<double>(4, 0.0));

    std::vector<double> buff =
        this->get_parameter("blue_side.start_pos").as_double_array();
    msg.startpos[0] = (float)buff[0];
    msg.startpos[1] = (float)buff[1];
    buff.clear();
    buff = this->get_parameter("blue_side.way_point").as_double_array();
    msg.waypoint[0] = (float)buff[0];
    msg.waypoint[1] = (float)buff[1];
    buff.clear();
    buff = this->get_parameter("blue_side.own_pos_x").as_double_array();
    for (int i = 0; i < 16; i++) {
      msg.ownx[i] = buff[i];
    }
    buff.clear();
    buff = this->get_parameter("blue_side.own_pos_y").as_double_array();
    for (int i = 0; i < 16; i++) {
      msg.owny[i] = buff[i];
    }
    buff.clear();
    buff = this->get_parameter("blue_side.cmn_pos_x").as_double_array();
    for (int i = 0; i < 9; i++) {
      msg.cmnx[i] = buff[i];
    }
    buff.clear();
    buff = this->get_parameter("blue_side.cmn_pos_y").as_double_array();
    for (int i = 0; i < 9; i++) {
      msg.cmny[i] = buff[i];
    }
    buff.clear();
    buff = this->get_parameter("blue_side.sht_pos_x").as_double_array();
    for (int i = 0; i < 9; i++) {
      msg.shtx[i] = buff[i];
    }
    buff.clear();
    buff = this->get_parameter("blue_side.sht_pos_y").as_double_array();
    for (int i = 0; i < 9; i++) {
      msg.shty[i] = buff[i];
    }
    buff.clear();
    buff = this->get_parameter("blue_side.own_field").as_double_array();
    for (int i = 0; i < 4; i++) {
      msg.ownfield[i] = buff[i];
    }
    buff.clear();
    buff = this->get_parameter("blue_side.cmn_field").as_double_array();
    for (int i = 0; i < 4; i++) {
      msg.cmnfield[i] = buff[i];
    }
    buff.clear();
    buff = this->get_parameter("blue_side.sht_field").as_double_array();
    for (int i = 0; i < 4; i++) {
      msg.shtfield[i] = buff[i];
    }
    buff.clear();
  }
  is_init = true;
}

void ParameterManager::send_param() {
  if (is_init) {
    param_pub_->publish(msg);
  }
}
void ParameterManager::save_param() {
  // save param to yaml
  RCLCPP_INFO(this->get_logger(), "Saving parameters to yaml...");
  // auto parameters = this->declare_parameter("parameters",
  // std::vector<int>()); for (auto &parameter : parameters) {
  //   RCLCPP_INFO(this->get_logger(), "Parameter: %d", parameter);
  // }
}

// void ParameterManager::get_state(
//     const principal_interfaces::msg::Parameters::SharedPtr msg) {
//   // get state from param_manager
//   RCLCPP_INFO(this->get_logger(), "Getting state from param_manager...");
// }

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParameterManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}