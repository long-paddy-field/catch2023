#include "auto_cmd_generator.hpp"

#include <iostream>

using namespace catch2023_principal;
using namespace std::chrono_literals;
using namespace std::placeholders;

AutoCmdGenerator::AutoCmdGenerator()
    : Node("auto_cmd_generator"), handle(auto_cmd) {
  state_command_subscription =
      this->create_subscription<principal_interfaces::msg::Statecommand>(
          "state_command", 10,
          [this](principal_interfaces::msg::Statecommand::SharedPtr stateCmd) {
            this->change_state_flag = stateCmd->phaze_change == 1;
            if (stateCmd->is_common == 1) {
              if (this->is_cmn == false) is_cmn = true;
            } else if (stateCmd->is_common == -1) {
              if (this->is_cmn == true) is_cmn = false;
            }
            this->shift_flag = stateCmd->shift;
          });
  is_auto_subscription = this->create_subscription<std_msgs::msg::Bool>(
      "is_auto", 10, [&, this](std_msgs::msg::Bool::SharedPtr isAuto) {
        this->is_auto = isAuto->data;
      });

  auto_command_publisher =
      this->create_publisher<principal_interfaces::msg::Movecommand>(
          "auto_move_command", 10);
  current_pos_subscription =
      this->create_subscription<principal_interfaces::msg::Movecommand>(
          "current_pos", 10,
          [&](principal_interfaces::msg::Movecommand::SharedPtr pos) {
            this->current_pos = pos;
          });
  param_sub = this->create_subscription<principal_interfaces::msg::Parameters>(
      "parameters", 10, std::bind(&AutoCmdGenerator::reflect_param, this, _1));
}
void AutoCmdGenerator::reflect_param(
    const principal_interfaces::msg::Parameters::SharedPtr msg) {
  if (!is_init) {
    side = msg->isred ? Side::Red : Side::Blue;
    WORKLOCATION location;
    location.own_area.push_back(
        std::make_pair(msg->startpos[0], msg->startpos[1]));
    location.cmn_area.push_back(
        std::make_pair(msg->waypoint[0], msg->waypoint[1]));
    for (int i = 0; i < sizeof(msg->ownx) / sizeof(float); i++) {
      location.own_area.push_back(std::make_pair(msg->ownx[i], msg->owny[i]));
    }
    for (int i = 0; i < sizeof(msg->cmnx) / sizeof(float); i++) {
      location.cmn_area.push_back(std::make_pair(msg->cmnx[i], msg->cmny[i]));
    }
    for (int i = 0; i < sizeof(msg->shtx) / sizeof(float); i++) {
      location.sht_area.push_back(std::make_pair(msg->shtx[i], msg->shty[i]));
    }
    for (int i = 0; i < sizeof(msg->stepperstate) / sizeof(float); i++) {
      location.stepper_state.push_back(msg->stepperstate[i]);
    }
    handle.init(side, msg->armoffset, msg->cmnoffset, msg->shtoffset, location);
  }
  is_init = true;
}

void AutoCmdGenerator::update() {
  while (rclcpp::ok()) {
    if (is_init) {
      if (is_auto) {
        auto_mode();
        // } else {
        //   manual_mode();
      }
    }
    rclcpp::spin_some(this->shared_from_this());
    rclcpp::sleep_for(100ms);
  }
}

void AutoCmdGenerator::auto_mode() {
  RCLCPP_INFO(this->get_logger(), "auto_cmd: auto_mode");
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
      if (shift_flag != 0) {
        // 左右十字が押されていたらずらすところ
        own_area_index += shift_flag;
        shift_flag = 0;
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
      if (shift_flag != 0) {
        // 左右十字が押されていたらずらすところ
        cmn_area_index += shift_flag;
        shift_flag = 0;
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
      break;
    case StateName::CatchOwn:
      handle.move_to(ZState::OwnCatch);
      if (has_arrived_z()) {
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
      handle.move_to(ZState::CmnCatch);
      if (has_arrived_z()) {
        handle.grasp(Area::Cmn, cmn_area_index);
        cmn_area_index += 1;
        spinsleep(2000000);
        past_state = state;
        if (cmn_area_index % 3 == 1) {
          // 一個目か、保持しているワークの数が3つであれば
          state = StateName::MoveToWaypoint;
        } else {
          if (is_cmn == false) {
            handle.move_to(Area::Cmn, cmn_area_index - 1,
                           (cmn_area_index - 1) % 3, ZState::CmnCatch, false);
            if (has_arrived()) {
              state = StateName::MoveToCmnWork;
            }
          } else if (shift_flag == 1) {
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
      if (change_state_flag || has_arrived()) {
        past_state = state;
        state = StateName::Release;
        change_state_flag = false;
      }
      break;
    case StateName::Release:
      handle.move_to(ZState::Shoot);  // まず下げる
      if (has_arrived_z()) {
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
          if (has_arrived_xy()) {
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
  RCLCPP_INFO(this->get_logger(), "auto_cmd is published: %d",
              static_cast<int>(state));
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

bool AutoCmdGenerator::has_arrived() {
  return has_arrived_xy() && has_arrived_z();
}

bool AutoCmdGenerator::has_arrived_xy() {
  float error = (auto_cmd.x - current_pos->x) * (auto_cmd.x - current_pos->x) +
                (auto_cmd.y - current_pos->y) * (auto_cmd.y - current_pos->y);
  return error < 0.005;
}

bool AutoCmdGenerator::has_arrived_z() {
  float error = (auto_cmd.z - current_pos->z) * (auto_cmd.z - current_pos->z);
  return error < 0.005;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoCmdGenerator>();
  node->update();
  rclcpp::shutdown();
  return 0;
}