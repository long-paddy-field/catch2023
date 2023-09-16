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
            this->change_area = stateCmd->is_common;
            this->next_choice = stateCmd->is_common;
            this->shift_flag = stateCmd->shift;
            this->store_flag = stateCmd->storeflag;
            this->homing_flag = stateCmd->homing && this->is_auto;
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
  joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, [&](sensor_msgs::msg::Joy::SharedPtr joy) {
        this->vertical = joy->axes[5];
        this->horizontal = joy->axes[4];
      });
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
    location.init_area = std::make_pair(msg->startpos[0], msg->startpos[1]);
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
                msg->handoffset, msg->fingeroffset, location);
  }
  is_init = true;
}

void AutoCmdGenerator::update() {
  while (rclcpp::ok()) {
    if (is_init) {
      if (is_auto) {
        if (!past_is_auto) {
          // 手動から自動に戻るときの復帰処理
          // if (state == StateName::MoveToOwnWork ||
          //     state == StateName::CatchOwn) {
          //   move_to_current_pos();
          //   state = StateName::CatchOwn;
          // } else if (state == StateName::MoveToCmnWork ||
          //            state == StateName::CatchCmn) {
          //   move_to_current_pos();
          //   state = StateName::CatchCmn;
          // } else if (state == StateName::MoveToBonus ||
          //            state == StateName::Release) {
          //   move_to_current_pos();
          //   state = StateName::Release;
          // }
        }
        auto_mode();
        // } else {
        //   manual_mode();
        past_is_auto = true;
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "auto_cmd-> own_index: " << own_area_index
                                     << ", cmn_index: " << cmn_area_index
                                     << ", sht_index: " << sht_area_index
                                     << ", state: " << static_cast<int>(state));
      } else {
        move_to_current_pos();
        past_is_auto = false;
      }
    }
    rclcpp::spin_some(this->shared_from_this());
    rclcpp::sleep_for(100ms);
  }
}

int ownref(int own_area_index) {
  if (own_area_index == 0 || own_area_index == 5 || own_area_index == 8 ||
      own_area_index == 11) {
    return 2;
  } else if (own_area_index == 1 || own_area_index == 4 ||
             own_area_index == 6 || own_area_index == 9 ||
             own_area_index == 12 || own_area_index == 14) {
    return 1;
  } else {
    return 0;
  }
}

void AutoCmdGenerator::auto_mode() {
  switch (state) {
    case StateName::Init:
      handle.move_to(Area::Init, 0, 0, ZState::OwnGiri);
      if (change_state_flag) {
        change_state_flag = false;
        change_state(StateName::MoveToWait);
      }
      break;
    case StateName::MoveToWait:
      if (shift_flag == 1) {
        if (own_area_index == 0) {
          own_area_index = 7;
        } else if (own_area_index == 7) {
          own_area_index = 10;
        } else if (own_area_index == 10) {
          own_area_index = 13;
        } else if (own_area_index == 13) {
          own_area_index = 15;
        } else if (own_area_index == 15) {
          own_area_index = 0;
        }
        shift_flag = 0;
      } else if (shift_flag == -1) {
        if (own_area_index == 0) {
          own_area_index = 15;
        } else if (own_area_index == 15) {
          own_area_index = 13;
        } else if (own_area_index == 13) {
          own_area_index = 10;
        } else if (own_area_index == 10) {
          own_area_index = 7;
        } else if (own_area_index == 7) {
          own_area_index = 0;
        }
        shift_flag = 0;
      }
      handle.move_to(Area::Own, 2, cmn_area_index, ZState::OwnGiri);
      if (change_area == 1) {
        if (own_area_index == 0) {
          own_area_index = 1;
          reverse_flag = false;
        } else if (own_area_index == 7) {
          if (own_progress[0]) {
            own_area_index = 3;
            reverse_flag = false;
          } else {
            reverse_flag = true;
          }
        } else if (own_area_index == 10) {
          if (own_progress[1]) {
            own_area_index = 5;
            reverse_flag = false;
          } else {
            reverse_flag = true;
          }
        } else if (own_area_index == 13) {
          if (own_progress[2]) {
            own_area_index = 8;
            reverse_flag = false;
          } else {
            reverse_flag = true;
          }
        } else if (own_area_index == 15) {
          if (own_progress[3]) {
            own_area_index = 11;
            reverse_flag = false;
          } else {
            reverse_flag = true;
          }
        }
        change_state(StateName::MoveToOwn);
      } else if (change_area == -1) {
        if (own_area_index == 0) {
          cmn_area_index = 0;
        } else if (own_area_index == 7) {
          cmn_area_index = 2;
        } else if (own_area_index == 10) {
          cmn_area_index = 4;
        } else if (own_area_index == 13) {
          cmn_area_index = 6;
        } else if (own_area_index == 15) {
          cmn_area_index = 8;
        }
        change_state(StateName::MoveToCmn);
      }
      if (change_state_flag) {
        change_state(StateName::OwnCatch);
        change_state_flag = false;
      }
      break;
    case StateName::OwnCatch:
      handle.move_to(ZState::OwnCatch);
      if (has_arrived_z() || change_state_flag) {
        handle.grasp(Area::Own, ownref(own_area_index));
        hold_count++;
        own_area_index = own_area_index + (reverse_flag ? -1 : 1);
        if (own_area_index < 0) {
          own_area_index = 15;
        } else if (own_area_index > 15) {
          own_area_index = 0;
        }
        spinsleep(300);
        if (past_state == StateName::MoveToWait || hold_count == 3) {
          change_state(
              StateName::OwnAbove);  // 一個目or3つ取ったらそのまま上がる
        } else {
          // 擦りながら移動（あとで）
          change_state(StateName::MoveOwnY);  // それ以外は擦り移動
        }
        change_state_flag = false;
        past_state = state;
      }
      break;
    case StateName::MoveOwnY: {
      float past_x = auto_cmd.x;
      int hand_index = ownref(own_area_index);
      handle.move_to(Area::Own, hand_index, own_area_index, ZState::OwnCatch);
      auto_cmd.x = past_x;
      if (has_arrived() || change_state_flag) {
        change_state_flag = false;
        change_state(StateName::OwnAbove);
      }
    } break;
    case StateName::MoveToOwn:
      handle.move_to(Area::Own, ownref(own_area_index), own_area_index,
                     ZState::OwnGiri);
      if (hold_count == 0) {
        if (change_area == 1) {
          change_state(StateName::MoveToStr);
          if (own_area_index < 3) {
            str_index = 0;
          } else if (own_area_index < 5) {
            str_index = 2;
          } else if (own_area_index < 8) {
            str_index = 4;
          } else if (own_area_index < 11) {
            str_index = 6;
          } else {
            str_index = 8;
          }
        } else if (change_area == -1) {
        }
      }
      if (has_arrived() && (change_state_flag || hold_count > 0)) {
        change_state(StateName::OwnCatch);
        change_state_flag = false;
      }
      break;
    case StateName::OwnAbove:
      handle.move_to(ZState::OwnGiri);
      if (has_arrived_z() || change_state_flag) {
        change_state_flag = false;
        if (past_state == StateName::MoveOwnY) {
          change_state(StateName::MoveToOwn);
        } else {
          if (reverse_flag) {
            change_state(StateName::MoveToRail);
          } else {
            if (hold_count != 3 || bns_count >= 6) {
              change_state(StateName::MoveToSht);
            } else {
              change_state(StateName::MoveToBns);
            }
          }
        }
      }
      break;
    case StateName::MoveToStr:
      handle.move_to(Area::Str, str_index % 3, str_index, ZState::OwnCatch);
      if (shift_flag != 0) {
        if (shift_flag == -1) {
          if (str_index == 0) {
            str_index = 8;
          } else {
            str_index--;
          }
        } else if (shift_flag == 1) {
          if (str_index == 8) {
            str_index = 0;
          } else {
            str_index++;
          }
        }
        shift_flag = 0;
      } else if (change_area == 1) {
        cmn_area_index = str_index;
        change_state(StateName::MoveToCmn);
      } else if (change_area == -1) {
        if (str_index < 2) {
          own_area_index = 1;
        } else if (str_index < 4) {
          own_area_index = 3;
        } else if (str_index < 6) {
          own_area_index = 5;
        } else if (str_index < 8) {
          own_area_index = 8;
        } else {
          own_area_index = 11;
        }
        change_state(StateName::MoveToOwn);
      }
      if (change_state_flag) {
        change_state(StateName::StrStore);
      }
      break;
    case StateName::StrStore:
      handle.move_to(ZState::OwnCatch);
      if (has_arrived_z() || change_state_flag) {
        handle.release();
        auto_cmd.hand[str_index % 3] = !auto_cmd.hand[str_index % 3];
        if (auto_cmd.hand[str_index % 3]) {
          hold_count++;
        } else {
          hold_count--;
        }
        change_state_flag = false;
        if (hold_count == 3) {
          change_state(StateName::MoveToSht);
        } else if (auto_cmd.hand[str_index % 3]) {
          change_state(StateName::MoveToStr);
        } else {
          cmn_area_index = str_index;
          change_state(StateName::MoveToCmn);
        }
      }
      break;
    case StateName::MoveToCmn:
      if (shift_flag != 0) {
        // 左右に移動、オーバーフローしたら反対側へ
        cmn_area_index += shift_flag;
        if (cmn_area_index < 0) {
          cmn_area_index = 8;
        } else if (cmn_area_index > 8) {
          cmn_area_index = 0;
        }
        shift_flag = 0;
        if (cmn_area_index < 0) {
          cmn_area_index = 8;
        } else if (cmn_area_index > 8) {
          cmn_area_index = 0;
        }
      }
      handle.move_to(Area::Cmn, cmn_area_index % 3, cmn_area_index, true);
      if (change_area == 1) {
        change_state(StateName::MoveToWait);
      } else if (change_area == -1) {
        change_state(StateName::MoveToStr);
      }
      if (has_arrived() || change_state_flag) {  // 本番は「かつ」にする
        change_state(StateName::CmnCatch);
        change_state_flag = false;
      }
      break;
    case StateName::CmnCatch:
      handle.move_to(ZState::CmnCatch);
      if (has_arrived_z() || change_state_flag) {
        handle.grasp(Area::Cmn, cmn_area_index % 3);
        hold_count++;
        spinsleep(300);
        change_state(StateName::CmnAbove);
      }
      break;
    case StateName::CmnAbove:
      handle.move_to(ZState::CmnAbove);
      if (has_arrived_z() || change_state_flag) {
        change_state(StateName::MoveToStr);
      }
      break;
    case StateName::MoveToRail:
      handle.move_to(Area::Own, 1, own_area_index);
      auto_cmd.y = 0.150;
      if (has_arrived() || change_state_flag) {
        if (hold_count != 3 || bns_count >= 6) {
          change_state(StateName::MoveToSht);
        } else {
          change_state(StateName::MoveToBns);
        }
      }
      break;
    case StateName::MoveToSht:
      handle.move_to(Area::Sht, 1, bns_count, ZState::ShtGiri, false);
      // 現在位置を取得してZを下げる
      if (current_pos->x < -0.5 && current_pos->y < 0.0) {
        handle.move_to(ZState::Shoot);
      }
      if (has_arrived() || change_state_flag) {
        if (hold_count != 3 || bns_count >= 6) {
          change_state(StateName::Release);
        } else {
          change_state(StateName::MoveToBns);
        }

        // ボーナスが全部埋まってたらBnsには行かないでリリースへ
      }
      break;
    case StateName::MoveToBns:
      handle.move_to(Area::Sht, 1, bns_count, ZState::ShtGiri, true);
      if (current_pos->x < 0.3 &&
          current_pos->y < bns_count * 1) {  // 後で下げる位置を確定
        handle.move_to(ZState::Shoot);
      }
      if (has_arrived() || change_state_flag) {
        change_state_flag = false;
        change_state(StateName::Release);
        bns_count++;
      }
      break;
    case StateName::Release:
      handle.release();
      hold_count = 0;
      handle.move_to(ZState::ShtAbove);
      if (has_arrived_z() || change_state_flag) {
        change_state_flag = false;
        change_state(StateName::MoveToWait);
      }
      break;
  }
}

// void AutoCmdGenerator::auto_mode() {
//   RCLCPP_INFO(this->get_logger(), "auto_cmd: auto_mode");
//   float past_x = auto_cmd.x;
//   own_area_index = own_area_index > 16 ? own_area_index - 16 :
//   own_area_index; cmn_area_index = cmn_area_index > 9 ? cmn_area_index - 9
//   : cmn_area_index; sht_area_index = sht_area_index > 10 ? sht_area_index -
//   10 : sht_area_index; switch (state) {
//     case StateName::Init:
//       RCLCPP_INFO(this->get_logger(), "auto_cmd: init");
//       handle.move_to(Area::Own, 1, 0,
//                      ZState::OwnGiri);  // 後で初期位置を代入
//       if (change_state_flag) {
//         past_state = state;
//         state = StateName::MoveToOwnWork;
//         change_state_flag = false;
//         own_area_index = 1;
//         cmn_area_index = 1;
//         sht_area_index = 1;
//       }
//       break;
//     case StateName::MoveToOwnWork:
//       RCLCPP_INFO(this->get_logger(), "auto_cmd: move_to_own_work");
//       if (shift_flag != 0) {
//         // 左右十字が押されていたらずらすところ
//         own_area_index += shift_flag;
//         shift_flag = 0;
//       }
//       if (own_area_index == 1) {
//         // 一個目のワークを取るときだけ例外
//         handle.move_to(Area::Own, side == Side::Red ? 0 : 2, 1,
//                        ZState::OwnGiri);
//       } else {
//         handle.move_to(Area::Own, (own_area_index + 1) % 3, own_area_index,
//                        ZState::OwnGiri);
//       }
//       auto_cmd.y += side == Side::Red ? (float)vertical * 0.015
//                                       : (float)vertical * (-0.015);
//       if (change_state_flag || has_arrived()) {
//         past_state = state;
//         state = StateName::CatchOwn;
//         change_state_flag = false;
//       }
//       break;
//     case StateName::MoveOwnY:
//       handle.move_to(Area::Own, (own_area_index + 1) % 3, own_area_index,
//                      ZState::OwnCatch);
//       auto_cmd.x = past_x;

//       if (change_state_flag || has_arrived()) {
//         past_state = state;
//         state = StateName::CatchAbove;
//         change_state_flag = false;
//       }
//       break;
//     case StateName::CatchOwn:
//       RCLCPP_INFO(this->get_logger(), "auto_cmd: catch_own");
//       handle.move_to(ZState::OwnCatch);
//       if (has_arrived_z() || change_state_flag) {
//         if (own_area_index == 1) {
//           handle.grasp(Area::Own,
//                        side == Side::Red ? 0 : 2);  // 初手自陣1つ取り
//         } else {
//           handle.grasp(Area::Own, (own_area_index + 1) % 3);  //
//           普通の自陣取り
//         }
//         spinsleep(300);
//         if (own_area_index == 1 || own_area_index % 3 == 1) {
//           // 一個目か、保持しているワークの数が3つであれば
//           own_area_index += 1;
//           past_state = state;
//           state = StateName::CatchAbove;
//         } else {
//           // 違ったら
//           own_area_index += 1;
//           past_state = state;
//           state = StateName::MoveOwnY;
//         }
//       }
//       break;
//     case StateName::CatchAbove:
//       RCLCPP_INFO(this->get_logger(), "auto_cmd: catch_above");
//       handle.move_to(ZState::OwnGiri);
//       if (has_arrived_z() || change_state_flag) {
//         change_state_flag = false;
//         if (past_state == StateName::MoveOwnY) {
//           past_state = state;
//           state = StateName::MoveToOwnWork;
//         } else if (past_state == StateName::CatchOwn) {
//           past_state = state;
//           state = StateName::MoveToShotBox;
//         }
//       }
//       break;
//     case StateName::MoveToWaypoint:
//       RCLCPP_INFO(this->get_logger(), "auto_cmd: move_to_waypoint");
//       if (past_state == StateName::Release) {
//         handle.move_to(Area::Cmn, 1, 0, ZState::ShtAbove);
//       } else if (past_state == StateName::CmnAbove) {
//         handle.move_to(Area::Cmn, 1, 0, ZState::ShtGiri);
//       }
//       if (change_state_flag || has_arrived()) {
//         if (past_state == StateName::Release) {
//           // シューティングボックス→共通エリア
//           past_state = state;
//           state = StateName::MoveToCmnWait;
//         } else {
//           // 共通エリア→シューティングボックス
//           past_state = state;
//           state = StateName::MoveToShotBox;
//         }
//         change_state_flag = false;
//       }
//       break;
//     case StateName::MoveToCmnWait:
//       if (shift_flag != 0) {
//         // 左右十字が押されていたらずらすところ
//         if (cmn_area_index != 1 && shift_flag < 0) {
//           cmn_area_index += shift_flag;
//         } else if (cmn_area_index != 9 && shift_flag > 0) {
//           cmn_area_index += shift_flag;
//         }
//         shift_flag = 0;
//       }
//       RCLCPP_INFO(this->get_logger(), "auto_cmd: move_to_cmn_wait");
//       handle.move_to(Area::Cmn, (hold_count) % 3, cmn_area_index,
//                      ZState::CmnAbove, false);
//       auto_cmd.rotate = store_flag ? 0 : (side == Side::Red ? 1 : -1);
//       if (has_arrived() || is_cmn) {  // TODO &&に変えないと実機で止まる
//         past_state = state;
//         state = StateName::MoveToCmnWork;
//       } else if ((has_arrived() && store_flag &&
//                   past_state == StateName::CatchAbove) ||
//                  change_state_flag) {
//         change_state_flag = false;
//         past_state = state;
//         state = StateName::CmnStore;
//       }
//       // TODO: 強制送還を追加
//       break;
//     case StateName::CmnStore:
//       RCLCPP_INFO(this->get_logger(), "auto_cmd: common_store");
//       handle.move_to(ZState::OwnCatch);
//       if (has_arrived() || change_state_flag) {
//         handle.release();
//         past_state = state;
//         state = StateName::MoveToCmnWait;
//         if (hold_count == 3) hold_count = 0;
//         change_state_flag = false;
//       }
//       break;
//     case StateName::MoveToCmnWork:
//       RCLCPP_INFO(this->get_logger(), "auto_cmd: move_to_cmn_work");
//       if (cmn_area_index == 9) {
//         handle.move_to(Area::Cmn, 2, cmn_area_index, ZState::CmnAbove,
//         true);

//       } else {
//         handle.move_to(Area::Cmn, hold_count % 3, cmn_area_index,
//                        ZState::CmnAbove, true);
//       }
//       auto_cmd.x += (side == Side::Red ? -1 : 1) * horizontal * 0.015;
//       if (change_state_flag || has_arrived()) {
//         // change_stateが押されたか、共通エリア上空に到着したら次へ
//         past_state = state;
//         state = StateName::CatchCmn;
//         change_state_flag = false;
//       }
//       break;

//     case StateName::CatchCmn:
//       RCLCPP_INFO(this->get_logger(), "auto_cmd: catch_cmn");
//       handle.move_to(ZState::CmnCatch);
//       if (has_arrived_z() || change_state_flag) {
//         change_state_flag = false;
//         handle.grasp(Area::Cmn, hold_count % 3);
//         spinsleep(1000);
//         hold_count++;
//         past_state = state;
//         state = StateName::CmnAbove;
//         // if (auto_cmd.hand[side == Side::Red ? hold_count % 3
//         //                                     : (2 - (hold_count % 3))] ==
//         //     false) {
//         //   handle.grasp(Area::Cmn, hold_count % 3);
//         //   spinsleep(1000);
//         // } else {
//         //   if (is_cmn && shift_flag != 0) {
//         //     if (cmn_area_index != 1 && shift_flag < 0) {
//         //       cmn_area_index += shift_flag;
//         //     } else if (cmn_area_index != 9 && shift_flag > 0) {
//         //       cmn_area_index += shift_flag;
//         //     }
//         //     shift_flag = 0;
//         //     hold_count++;
//         //     past_state = state;
//         //     state = StateName::MoveToCmnWork;
//         //   } else if (!is_cmn) {
//         //     hold_count++;
//         //     past_state = state;
//         //     state = StateName::CmnAbove;
//         //   }
//         // }
//       }
//       break;
//     case StateName::CmnAbove:
//       RCLCPP_INFO(this->get_logger(), "auto_cmd: CmnAbove");
//       handle.move_to(ZState::CmnAbove);
//       if (has_arrived_z() || change_state_flag) {
//         change_state_flag = false;
//         if (homing_flag) {
//           homing_flag = false;
//           cmn_area_index++;
//           past_state = state;
//           state = StateName::MoveToWaypoint;
//         } else if (is_cmn && shift_flag != 0) {
//           if (cmn_area_index != 1 && shift_flag < 0) {
//             cmn_area_index += shift_flag;
//           } else if (cmn_area_index != 9 && shift_flag > 0) {
//             cmn_area_index += shift_flag;
//           }
//           shift_flag = 0;
//           past_state = state;
//           state = StateName::MoveToCmnWork;
//         } else if (!is_cmn) {
//           past_state = state;
//           state = StateName::MoveToCmnWait;
//           cmn_area_index++;
//         } else {
//           auto_cmd.rotate = store_flag ? 0 : (side == Side::Red ? 1 : -1);
//         }
//       }
//       // handle.move_to(Area::Cmn, hold_count - 1, cmn_area_index,
//       //                ZState::CmnAbove, true);
//       // if (has_arrived_z() || change_state_flag) {
//       //   change_state_flag = false;
//       //   if (hold_count == 3) {
//       //     past_state = state;
//       //     state = StateName::MoveToWaypoint;
//       //   } else {
//       //     past_state = state;
//       //     state = StateName::MoveToCmnWait;
//       //   }
//       // }
//       break;
//     case StateName::MoveToShotBox:
//       RCLCPP_INFO(this->get_logger(), "auto_cmd: move_to_shoot");
//       handle.move_to(Area::Sht, 1, 0, ZState::ShtGiri, true);
//       if (change_state_flag || has_arrived()) {
//         past_state = state;
//         state = StateName::MoveToRelease;
//         change_state_flag = false;
//       }
//       break;
//     case StateName::MoveToRelease:
//       RCLCPP_INFO(this->get_logger(), "auto_cmn: move_to_release");
//       handle.move_to(Area::Sht, 1, sht_area_index, ZState::ShtGiri, false);
//       if (change_state_flag || has_arrived()) {
//         change_state_flag = false;
//         if (sht_area_index == 1 || sht_area_index > 7) {
//           past_state = state;
//           state = StateName::Release;
//         } else {
//           past_state = state;
//           state = StateName::ShtTsukemen;
//         }
//       }
//       break;
//     case StateName::ShtTsukemen:
//       RCLCPP_INFO(this->get_logger(), "auto_cmn: Shoot_Tsukemen");
//       handle.move_to(ZState::ShtTsuke);
//       if (change_state_flag || has_arrived_z()) {
//         change_state_flag = false;
//         past_state = state;
//         state = StateName::MoveToBonus;
//       }
//       break;
//     case StateName::MoveToBonus:
//       RCLCPP_INFO(this->get_logger(), "auto_cmd: move_to_bonus");
//       handle.move_to(Area::Sht, 1, sht_area_index, ZState::Shoot, true);
//       if (change_state_flag || has_arrived()) {
//         change_state_flag = false;
//         past_state = state;
//         state = StateName::Release;
//       }
//       break;
//     case StateName::Release:
//       RCLCPP_INFO(this->get_logger(), "auto_cmd: release");
//       if (auto_cmd.hand[0] || auto_cmd.hand[1] || auto_cmd.hand[2]) {
//         handle.release();
//         sht_area_index++;
//         hold_count = 0;
//       } else {
//         handle.move_to(ZState::ShtAbove);
//         if (has_arrived_z() || change_state_flag) {
//           change_state_flag = false;
//           if (next_choice == 1) {
//             past_state = state;
//             state = StateName::MoveToWaypoint;
//             is_cmn = false;
//           } else if (next_choice == -1) {
//             past_state = state;
//             state = StateName::MoveToOwnWork;
//           }
//         }
//       }
//       break;
//   }
//   // auto_command_publisher->publish(auto_cmd);

//   // rclcpp::sleep_for(100ms);
// }

void AutoCmdGenerator::manual_mode() {
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

void AutoCmdGenerator::move_to_current_pos() {
  if (current_pos.get() == nullptr) {
    return;
  }
  auto_cmd.x = current_pos->x;
  auto_cmd.y = current_pos->y;
  auto_cmd.z = current_pos->z;
}

void AutoCmdGenerator::change_state(StateName state) {
  past_state = this->state;
  this->state = state;
  change_state_flag = false;
}

int progress_ref(int own_area_index) {
  if (own_area_index == 1 || own_area_index == 2) {
    return 0;
  } else if (own_area_index == 0 || own_area_index == 3 ||
             own_area_index == 4) {
    return 1;
  } else if (own_area_index == 5 || own_area_index == 6 ||
             own_area_index == 7) {
    return 2;
  } else if (own_area_index == 8 || own_area_index == 9 ||
             own_area_index == 10) {
    return 3;
  } else if (own_area_index == 11 || own_area_index == 12 ||
             own_area_index == 13) {
    return 4;
  } else if (own_area_index == 14 || own_area_index == 15) {
    return 5;
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoCmdGenerator>();
  node->update();
  rclcpp::shutdown();
  return 0;
}