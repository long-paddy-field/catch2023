#pragma once
#include <chrono>
#include <memory>
#include <tuple>
#include <vector>

#include "principal_interfaces/msg/movecommand.hpp"
#include "principal_interfaces/msg/parameters.hpp"
#include "principal_interfaces/msg/statecommand.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "state.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using namespace catch2023_principal;

namespace catch2023_principal {
class AutoCmdGenerator : public rclcpp::Node {
 public:
  AutoCmdGenerator();
  ~AutoCmdGenerator(){};

  void update();

 private:
  std::pair<float, float> start_pos;
  std::pair<float, float> way_point;
  std::vector<std::pair<float, float>> own_area;  // 自陣エリアの目標値
  std::vector<std::pair<float, float>> cmn_area;  // 自陣エリアの目標値
  std::vector<std::pair<float, float>> str_area;  // 自陣エリアの目標値
  std::vector<std::pair<float, float>> sht_area;  // 自陣エリアの目標値

  int own_area_index = 0;  // 自陣エリアの目標値のインデックス
  int cmn_area_index = 0;  // 共有エリアの目標値のインデックス
  int sht_area_index = 0;  // 射撃エリアの目標値のインデックス

  int hold_count = 0;  // ホールドしたワークの数

  bool change_state_flag = false;
  StateName state = StateName::Init;       // 現在の状態
  StateName past_state = StateName::Init;  // 一つ前の状態
  Side side = Side::Red;                   // 現在のサイド
  bool is_cmn = false;  // 共通エリアに侵入できるかどうか
  bool is_init = false;  // パラメタの読み取りが終わったかどうか
  int shift_flag = 0;
  int next_choice = 0;  // リリースしたあとどっちに行くか
  int vertical = 0;
  int horizontal = 0;
  bool store_flag = false;
  bool homing_flag = false;
  int change_area = 0;
  bool reverse_flag = false;
  int bns_count = 0;
  bool own_progress[6] = {false, false, false, false, false, false};
  int rayer_count = 0;
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
  rclcpp::Subscription<principal_interfaces::msg::Parameters>::SharedPtr
      param_sub;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  GeneralCommand handle;

  void auto_mode();
  void manual_mode();
  void spinsleep(int ms);
  void move_to_current_pos();
  void reflect_param(
      const principal_interfaces::msg::Parameters::SharedPtr msg);
  bool has_arrived();
  bool has_arrived_xy();
  bool has_arrived_z();
  bool is_auto;
  bool past_is_auto;

  void change_state(StateName next_state);
};
}  // namespace catch2023_principal