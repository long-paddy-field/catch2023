#pragma once
#include <vector>

#include "principal_interfaces/msg/movecommand.hpp"
namespace catch2023_principal {
enum class StateName {
  Init,
  MoveToOwnWork,
  MoveToCmnWork,
  MoveToWaypoint,
  CatchOwn,
  CatchCmn,
  MoveToShoot,
  Release,
};
enum class Side { Blue, Red };
enum class Area { Own, Cmn, Sht };
enum class ZState { Trans, OwnCatch, OwnTrans, CmnCatch, CmnTrans, Shoot };

struct LOCATION {
  float x;
  float y;
  float z;
};  // 位置

struct WORKLOCATION {
  std::vector<std::pair<float, float>> own_area;  // 自陣エリアのワークxy座標
  std::vector<std::pair<float, float>> cmn_area;  // 共有エリアのワークxy座標
  std::vector<std::pair<float, float>> sht_area;  // ボーナスエリアのxy座標
  std::vector<float> stepper_state;  // 自陣エリアのワークz座標
};

class GeneralCommand {
 public:
  GeneralCommand(principal_interfaces::msg::Movecommand& _msg) : msg(_msg){};
  void init(Side _side, float _arm_offset, float _cmn_offset, float _sht_offset,
            WORKLOCATION _location);
  void move_to(Area area, int num, int hand, ZState z_state,
               bool is_cmn = false);
  void move_to(ZState z_state);
  void grasp(Area area, int n);
  void release();

 private:
  principal_interfaces::msg::Movecommand& msg;
  WORKLOCATION
  location;  // 行くべき座標の塊。own_area[0]は競技開始位置、cmn_area[0]は共通エリアでのウェイポイントであることに注意
  Side side;
  bool is_init = false;
  float arm_offset;
  float cmn_offset;
  float sht_offset;
};

}  // namespace catch2023_principal