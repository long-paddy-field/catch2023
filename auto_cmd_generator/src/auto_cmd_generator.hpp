#pragma once
#include <memory>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"

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
};
}  // namespace catch2023_principal