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
            this->next_choice = stateCmd->is_common;
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
  timer_ = this->create_wall_timer(100ms, [this]() {
    this->auto_command_publisher->publish(auto_cmd);
    RCLCPP_INFO(this->get_logger(), "auto_cmd is published: %d",
                static_cast<int>(state));
  });
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
    for (size_t i = 0; i < sizeof(msg->ownx) / sizeof(float); i++) {
      location.own_area.push_back(std::make_pair(msg->ownx[i], msg->owny[i]));
    }
    for (size_t i = 0; i < sizeof(msg->cmnx) / sizeof(float); i++) {
      location.cmn_area.push_back(std::make_pair(msg->cmnx[i], msg->cmny[i]));
    }
    for (size_t i = 0; i < sizeof(msg->shtx) / sizeof(float); i++) {
      location.sht_area.push_back(std::make_pair(msg->shtx[i], msg->shty[i]));
    }
    for (size_t i = 0; i < sizeof(msg->stepperstate) / sizeof(float); i++) {
      location.stepper_state.push_back(msg->stepperstate[i]);
    }
    handle.init(side, msg->armoffset, msg->cmnoffset, msg->shtoffset,
                msg->handoffset, location);
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
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "auto_cmd-> own_index: " << own_area_index
                                     << ", cmn_index: " << cmn_area_index
                                     << ", sht_index: " << sht_area_index
                                     << ", state: " << static_cast<int>(state));
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
      RCLCPP_INFO(this->get_logger(), "auto_cmd: init");
      handle.move_to(Area::Own, 1, 0,
                     ZState::OwnGiri);  // 後で初期位置を代入
      if (change_state_flag) {
        past_state = state;
        state = StateName::MoveToOwnWork;
        change_state_flag = false;
        own_area_index = 1;
        cmn_area_index = 1;
        sht_area_index = 1;
      }
      break;
    case StateName::MoveToOwnWork:
      RCLCPP_INFO(this->get_logger(), "auto_cmd: move_to_own_work");
      if (shift_flag != 0) {
        // 左右十字が押されていたらずらすところ
        own_area_index += shift_flag;
        shift_flag = 0;
      }
      if (own_area_index == 1) {
        // 一個目のワークを取るときだけ例外
        handle.move_to(Area::Own, side == Side::Red ? 0 : 2, 1,
                       ZState::OwnGiri);
      } else {
        handle.move_to(Area::Own, own_area_index % 3, own_area_index,
                       ZState::OwnGiri);
      }
      if (change_state_flag || has_arrived()) {
        past_state = state;
        state = StateName::CatchOwn;
        change_state_flag = false;
      }
      break;
    case StateName::CatchOwn:
      RCLCPP_INFO(this->get_logger(), "auto_cmd: catch_own");
      handle.move_to(ZState::OwnCatch);
      if (has_arrived_z() || change_state_flag) {
        if (own_area_index == 1) {
          handle.grasp(Area::Own, side == Side::Red ? 0 : 2);
        } else {
          handle.grasp(Area::Own, own_area_index % 3);
        }
        spinsleep(2000);
        change_state_flag = false;
        if (own_area_index == 1 || own_area_index % 3 == 1) {
          // 一個目か、保持しているワークの数が3つであれば
          own_area_index += 1;
          past_state = state;
          state = StateName::MoveToShotBox;
        } else {
          // 違ったら
          own_area_index += 1;
          past_state = state;
          state = StateName::MoveToOwnWork;
        }
      }
      break;
    case StateName::MoveToWaypoint:
      RCLCPP_INFO(this->get_logger(), "auto_cmd: move_to_waypoint");
      if (past_state == StateName::Release) {
        handle.move_to(Area::Cmn, 1, 0, ZState::ShtAbove);
      } else if (past_state == StateName::CmnAbove) {
        handle.move_to(Area::Cmn, 1, 0, ZState::ShtGiri);
      }
      if (change_state_flag || has_arrived()) {
        if (past_state == StateName::Release) {
          // シューティングボックス→共通エリア
          past_state = state;
          state = StateName::MoveToCmnWait;
        } else {
          // 共通エリア→シューティングボックス
          past_state = state;
          state = StateName::MoveToShotBox;
        }
        change_state_flag = false;
      }
      break;
    case StateName::MoveToCmnWait:
      if (shift_flag != 0) {
        // 左右十字が押されていたらずらすところ
        if (cmn_area_index != 1 && shift_flag < 0) {
          cmn_area_index += shift_flag;
        } else if (cmn_area_index != 9 && shift_flag > 0) {
          cmn_area_index += shift_flag;
        }
        shift_flag = 0;
      }
      RCLCPP_INFO(this->get_logger(), "auto_cmd: move_to_cmn_wait");
      handle.move_to(Area::Cmn, (hold_count) % 3, cmn_area_index,
                     ZState::CmnAbove, false);
      if (has_arrived() || is_cmn) {  // TODO &&に変えないと実機で止まる
        past_state = state;
        state = StateName::MoveToCmnWork;
      }
      // TODO: 強制送還を追加
      break;
    case StateName::MoveToCmnWork:
      RCLCPP_INFO(this->get_logger(), "auto_cmd: move_to_cmn_work");
      if (cmn_area_index == 9) {
        handle.move_to(Area::Cmn, 2, cmn_area_index, ZState::CmnAbove, true);

      } else {
        handle.move_to(Area::Cmn, hold_count % 3, cmn_area_index,
                       ZState::CmnAbove, true);
      }
      if (change_state_flag || has_arrived()) {
        // change_stateが押されたか、共通エリア上空に到着したら次へ
        past_state = state;
        state = StateName::CatchCmn;
        change_state_flag = false;
      }
      break;

    case StateName::CatchCmn:
      RCLCPP_INFO(this->get_logger(), "auto_cmd: catch_cmn");
      handle.move_to(ZState::CmnCatch);
      if (has_arrived_z() || change_state_flag) {
        change_state_flag = false;
        if (auto_cmd.hand[side == Side::Red ? hold_count % 3
                                            : (2 - (hold_count % 3))] ==
            false) {
          handle.grasp(Area::Cmn, hold_count % 3);
          spinsleep(2000);
        } else {
          if (is_cmn && shift_flag != 0) {
            if (cmn_area_index != 1 && shift_flag < 0) {
              cmn_area_index += shift_flag;
            } else if (cmn_area_index != 9 && shift_flag > 0) {
              cmn_area_index += shift_flag;
            }
            shift_flag = 0;
            hold_count++;
            past_state = state;
            state = StateName::MoveToCmnWork;
            past_state = state;
          } else {
            hold_count++;
            past_state = state;
            state = StateName::CmnAbove;
          }
        }
      }
      break;
    case StateName::CmnAbove:
      RCLCPP_INFO(this->get_logger(), "auto_cmd: CmnAbove");
      handle.move_to(Area::Cmn, hold_count - 1, cmn_area_index,
                     ZState::CmnAbove, true);
      if (has_arrived_z() || change_state_flag) {
        change_state_flag = false;
        if (hold_count == 3) {
          past_state = state;
          state = StateName::MoveToWaypoint;
        } else {
          past_state = state;
          state = StateName::MoveToCmnWait;
        }
      }
      break;
    case StateName::MoveToShotBox:
      RCLCPP_INFO(this->get_logger(), "auto_cmd: move_to_shoot");
      handle.move_to(Area::Sht, 1, 0, ZState::ShtGiri);
      if (change_state_flag || has_arrived()) {
        past_state = state;
        state = StateName::MoveToRelease;
        change_state_flag = false;
      }
      break;
    case StateName::MoveToRelease:
      RCLCPP_INFO(this->get_logger(), "auto_cmn: move_to_release");
      handle.move_to(Area::Sht, 1, sht_area_index, ZState::Shoot, false);
      if (change_state_flag || has_arrived()) {
        change_state_flag = false;
        if (sht_area_index == 1 || sht_area_index > 7) {
          past_state = state;
          state = StateName::Release;
        } else {
          past_state = state;
          state = StateName::MoveToBonus;
        }
      }
      break;
    case StateName::MoveToBonus:
      RCLCPP_INFO(this->get_logger(), "auto_cmd: move_to_bonus");
      handle.move_to(Area::Sht, 1, sht_area_index, ZState::Shoot, true);
      if (change_state_flag || has_arrived()) {
        change_state_flag = false;
        past_state = state;
        state = StateName::Release;
      }
      break;
    case StateName::Release:
      RCLCPP_INFO(this->get_logger(), "auto_cmd: release");
      if (auto_cmd.hand[0] || auto_cmd.hand[1] || auto_cmd.hand[2]) {
        handle.release();
        sht_area_index++;
        hold_count = 0;
      } else {
        handle.move_to(ZState::ShtAbove);
        if (has_arrived_z() || change_state_flag) {
          change_state_flag = false;
          if (next_choice == 1) {
            past_state = state;
            state = StateName::MoveToWaypoint;
            is_cmn = false;
          } else if (next_choice == -1) {
            past_state = state;
            state = StateName::MoveToOwnWork;
          }
        }
      }
      break;
  }
  // auto_command_publisher->publish(auto_cmd);

  // rclcpp::sleep_for(100ms);
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
  // return has_arrived_xy() && has_arrived_z();
  return false;
}

bool AutoCmdGenerator::has_arrived_xy() {
  float error =
      sqrt((auto_cmd.x - current_pos->x) * (auto_cmd.x - current_pos->x)) +
      sqrt((auto_cmd.y - current_pos->y) * (auto_cmd.y - current_pos->y));
  return error < 0.005;
}

bool AutoCmdGenerator::has_arrived_z() {
  float error =
      sqrt((auto_cmd.z - current_pos->z) * (auto_cmd.z - current_pos->z));
  // return error < 0.005;
  return false;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoCmdGenerator>();
  node->update();
  rclcpp::shutdown();
  return 0;
}