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
      handle.move_to(0, 0, 0);  // 後で初期位置を代入
      auto_cmd.rotate = (side == Side::Blue) ? 1 : -1;
      if (change_state_flag) {
        state = StateName::MoveToOwnWork;
        change_state_flag = false;
      }
      break;
    case StateName::MoveToOwnWork:
      if (true) {
        // 左右十字が押されていたらずらすところ
      }
      handle.move_to(0, 0, 0);  // 後で
      if (change_state_flag || has_arrived()) {
        state = StateName::CatchOwn;
        change_state_flag = false;
      }
      break;
    case StateName::MoveToCmnWork:
      if (true) {
        // 左右十字が押されていたらずらすところ
      }
      if (true) {
        // 上下キーが押されたらずらすところ
      }
      handle.move_to(0, 0, 0);
      if (change_state_flag || (has_arrived() && true)) {
        // change_stateが押されたか、共通エリア上空に到着したら次へ
        state = StateName::CatchCmn;
        change_state_flag = false;
      }
      break;
    case StateName::CatchOwn:
      if (!has_arrived()) {
        handle.move_to(0, 0, 0);
      } else {
        handle.grasp(0);
        spinsleep(2000000);
        if (true) {
          // 一個目か、保持しているワークの数が3つであれば
          state = StateName::MoveToShoot;
        } else {
          // 違ったら
          state = StateName::MoveToOwnWork;
          // カウント増やしとく
        }
      }
      break;
    case StateName::CatchCmn:
      if (!has_arrived()) {
        handle.move_to(0, 0, 0);
      } else {
        handle.grasp(0);
        if (true) {
          // 一個目か、保持しているワークの数が3つであれば
          state = StateName::MoveToShoot;
        } else {
          if (true) {
            // 下キーが押されたら

          } else if (false) {
            // 右キーが押されたら
          }

          // 違ったら
          state = StateName::MoveToCmnWork;
          // カウント増やしとく
        }
      }
      break;
    case StateName::MoveToShoot:
      if (true) {
        // シュートした回数が偶数なら
        handle.move_to(0, 0, 0);  // ボーナスエリア上空へ

      } else {
        // シュートした回数が奇数なら
        handle.move_to(0, 0, 0);  // ボーナスエリアの手前へ
      }
      break;
    case StateName::Release:
      if (true) {
        // シュートした回数が偶数なら
        handle.move_to(0, 0, 0);  // 下げて
      } else {
        // シュートした回数が奇数なら
        handle.move_to(0, 0, 0);  // 平行移動
      }
      if (change_state_flag || has_arrived()) {
        handle.release();
        if (true) {
          // ２〜９個目なら
          state = StateName::MoveToCmnWork;
        } else {
          state = StateName::MoveToOwnWork;
        }
      }
      break;
  }
  auto_command_publisher->publish(auto_cmd);
  rclcpp::sleep_for(100ms);
}

void manual_mode(){
  // 現在のアームの位置がどこにあるのかを計算し、どのステートに相当するかを推測
  // 


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