#include "auto_cmd_generator.hpp"

#include <iostream>

using namespace catch2023_principal;
using namespace std::chrono_literals;

AutoCmdGenerator::AutoCmdGenerator() : Node("auto_cmd_generator") {
  state_command_subscription =
      this->create_subscription<principal_interfaces::msg::Statecommand>(
          "state_command", 10,
          [&](principal_interfaces::msg::Statecommand::SharedPtr stateCmd) {

          });
  is_auto_subscription = this->create_subscription<std_msgs::msg::Bool>(
      "is_auto", 10, [&, this](std_msgs::msg::Bool::SharedPtr isAuto) {});

  auto_command_publisher =
      this->create_publisher<principal_interfaces::msg::Movecommand>(
          "auto_move_command", 10);
  current_pos_subscription =
      this->create_subscription<principal_interfaces::msg::Movecommand>(
          "current_pos", 10,
          [&](principal_interfaces::msg::Movecommand::SharedPtr pos) {
            this->current_pos = pos;
          });

  this->declare_parameter("side", "blue");
  side = this->get_parameter("side").as_string() == "blue" ? Side::Blue
                                                           : Side::Red;

  this->declare_parameter("config_folder", std::string(__FILE__) + "../config");
  std::string config_folder = this->get_parameter("config_folder").as_string();
}

void AutoCmdGenerator::update() {
  while (rclcpp::ok()) {
    if (is_auto) {
      auto_mode();
    } else {
      manual_mode();
    }
    rclcpp::sleep_for(100ms);
  }
}

void AutoCmdGenerator::auto_mode() {
  switch (state) {
    case StateName::Init:
      handle.move_to(Area::Own, 0, 1,
                     ZState::Trans);  // 後で初期位置を代入
      if (change_state_flag) {
        past_state = state;
        state = StateName::MoveToOwnWork;
        change_state_flag = false;
      }
      break;
    case StateName::MoveToOwnWork:
      if (true) {
        // 左右十字が押されていたらずらすところ
        own_area_index += 1;
      }
      if (own_area_index == 1) {
        // 一個目のワークを取るときだけ例外
        if (side == Side::Blue) {
          handle.move_to(Area::Own, 2, own_area_index % 3, ZState::Trans);
        } else {
          handle.move_to(Area::Own, 0, own_area_index % 3, ZState::Trans);
        }
      } else {
        handle.move_to(Area::Own, own_area_index, own_area_index % 3,
                       ZState::Trans);
      }
      if (change_state_flag || has_arrived()) {
        past_state = state;
        state = StateName::CatchOwn;
        change_state_flag = false;
      }
      break;
    case StateName::MoveToCmnWork:
      if (true) {
        // 左右十字が押されていたらずらすところ
        cmn_area_index += 1;
      }
      handle.move_to(Area::Cmn, cmn_area_index, cmn_area_index % 3,
                     ZState::Trans, is_cmn);
      if (change_state_flag || (has_arrived() && is_cmn)) {
        // change_stateが押されたか、共通エリア上空に到着したら次へ
        past_state = state;
        state = StateName::CatchCmn;
        change_state_flag = false;
      }
      break;
    case StateName::MoveToWaypoint:
      handle.move_to(Area::Cmn, 0, 0, ZState::Trans);
      if (change_state_flag || has_arrived()) {
        if (past_state == StateName::Release) {
          // シューティングボックス→共通エリア
          state = StateName::MoveToCmnWork;
        } else {
          // 共通エリア→シューティングボックス
          state = StateName::MoveToShoot;
        }
        change_state_flag = false;
      }
    case StateName::CatchOwn:
      if (!has_arrived()) {
        handle.move_to(ZState::OwnCatch);
      } else {
        handle.grasp(Area::Own, own_area_index % 3);
        own_area_index += 1;
        spinsleep(2000000);
        if (own_area_index == 1 || own_area_index % 3 == 0) {
          // 一個目か、保持しているワークの数が3つであれば
          state = StateName::MoveToShoot;
        } else {
          // 違ったら
          state = StateName::MoveToOwnWork;
        }
      }
      break;
    case StateName::CatchCmn:
      if (!has_arrived()) {
        handle.move_to(ZState::CmnCatch);
      } else {
        handle.grasp(Area::Cmn, cmn_area_index);
        cmn_area_index += 1;
        spinsleep(2000000);
        past_state = state;
        if (cmn_area_index % 3 == 1) {
          // 一個目か、保持しているワークの数が3つであれば
          state = StateName::MoveToWaypoint;
        } else {
          if (true) {
            handle.move_to(Area::Cmn, cmn_area_index - 1,
                           (cmn_area_index - 1) % 3, ZState::CmnCatch, false);
            is_cmn = false;
            if (has_arrived()) {
              state = StateName::MoveToCmnWork;
            }
          } else if (false) {
            // 右キーが押されたら
            state = StateName::MoveToCmnWork;
          }
        }
      }
      break;
    case StateName::MoveToShoot:
      if (sht_area_index % 2 == 0) {
        // シュートした回数が偶数なら
        handle.move_to(Area::Sht, sht_area_index, 0, ZState::Trans,
                       false);  // ボーナスエリアの手前へ
      } else {
        // シュートした回数が奇数なら
        handle.move_to(Area::Sht, sht_area_index, 0, ZState::Trans,
                       true);  // ボーナスエリアの上空へ
      }
      if (change_state_flag || has_arrived) {
        past_state = state;
        state = StateName::Release;
        change_state_flag = false;
      }
      break;
    case StateName::Release:
      handle.move_to(ZState::Shoot);  // まず下げる
      if (has_arrived()) {
        if (sht_area_index % 2 == 0) {
          // シュートした回数が偶数なら
          handle.release();
          sht_area_index += 1;
          past_state = state;
          if (cmn_area_index <= 10) {
            state = StateName::MoveToWaypoint;
          } else {
            state = StateName::MoveToOwnWork;
          }
        } else {
          // シュートした回数が奇数なら
          handle.move_to(Area::Sht, sht_area_index, 0, ZState::Trans,
                         true);  // ボーナスエリアの上空へ
          if (has_arrived()) {
            handle.release();
            sht_area_index += 1;
            past_state = state;
            if (cmn_area_index <= 10) {
              state = StateName::MoveToWaypoint;
            } else {
              state = StateName::MoveToOwnWork;
            }
          }
        }
      }
      break;
  }
  auto_command_publisher->publish(auto_cmd);
  rclcpp::sleep_for(100ms);
}

void manual_mode() {
  // 現在のアームの位置がどこにあるのかを計算し、どのステートに相当するかを推測
  // そのステートに相当するところに移動する
}

void AutoCmdGenerator::spinsleep(int ms) {
  int64_t t = rclcpp::Clock().now().nanoseconds();
  while (rclcpp::Clock().now().nanoseconds() - t < ms * 1e6) {
    rclcpp::spin_some(this->shared_from_this());
    rclcpp::sleep_for(10ms);
  }
}
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoCmdGenerator>());
  rclcpp::shutdown();
  return 0;
}