#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <string>

#include "principal_interfaces/msg/joycommand.hpp"
#include "rclcpp/rclcpp.hpp"

namespace catch2023_principal {
// ノードクラス兼ステート管理クラス
class CoreCatch : public rclcpp::Node {
 public:
  // コンストラクタ
  CoreCatch();
  // デストラクタ
  ~CoreCatch(){};

 private:
  // メンバ関数
  void update();
};
}  // namespace catch2023_principal