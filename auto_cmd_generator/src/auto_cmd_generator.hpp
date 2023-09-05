#pragma once
#include <chrono>
#include <memory>
#include <tuple>
#include <vector>

#include "principal_interfaces/msg/movecommand.hpp"
#include "principal_interfaces/msg/statecommand.hpp"
#include "rclcpp/rclcpp.hpp"
#include "state.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using namespace catch2023_principal;

namespace catch2023_principal {
class AutoCmdGenerator : public rclcpp::Node {
 public:
  AutoCmdGenerator();
  ~AutoCmdGenerator(){};

 private:
  std::vector<std::tuple<float, float, bool>> own_area;  // 自陣エリアの目標値
  std::vector<std::tuple<float, float, bool>> cmn_area;  // 共有エリアの目標値
  std::vector<std::tuple<float, float, bool>> sht_area;  // 射撃エリアの目標値

  int own_area_index = 0;  // 自陣エリアの目標値のインデックス
  int cmn_area_index = 0;  // 共有エリアの目標値のインデックス
  int sht_area_index = 0;  // 射撃エリアの目標値のインデックス

  int hold_count = 0;  // ホールドしたワークの数
  bool change_state_flag = false;
  StateName state = StateName::Init;  // 現在の状態
  Side side = Side::Blue;             // 現在のサイド

  principal_interfaces::msg::Movecommand auto_cmd;
  principal_interfaces::msg::Movecommand::SharedPtr current_pos;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<principal_interfaces::msg::Statecommand>::SharedPtr
      state_command_subscription;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_auto_subscription;
  rclcpp::Publisher<principal_interfaces::msg::Movecommand>::SharedPtr
      auto_command_publisher;
  rclcpp::Subscription<principal_interfaces::msg::Movecommand>::SharedPtr
      current_pos_subscription;

  GeneralCommand handle;

  void update();
  void auto_mode();
  void manual_mode();
  void spinsleep(int ms);
  void reflect_param();
  bool has_arrived();
  bool is_auto;
};
}  // namespace catch2023_principal