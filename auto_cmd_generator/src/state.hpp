#pragma once
#include <vector>

#include "principal_interfaces/msg/movecommand.hpp"
namespace catch2023_principal {
enum class StateName {
  Init,
  MoveToWait,
  MoveToOwn,
  OwnCatch,
  OwnAbove,
  MoveOwnY,
  MoveToStr,
  StrStore,
  MoveToCmn,
  CmnCatch,
  CmnAbove,
  MoveToSht,
  MoveToSingle,
  MoveToTriple,
  MoveToWall,
  Release,

  // Init,           // 初期状態(高さは自陣ギリ)
  // MoveToWait,     // 待機状態(高さは自陣ギリ）
  // MoveToOwnWork,  // 自陣エリアへ移動（高さは自陣ギリ）
  // MoveOwnY,  // 自陣エリア内擦りながらY方向へ移動（高さは自陣キャチ)
  // CatchOwn,    // 自陣の把持（高さは自陣キャチ）
  // CatchAbove,  // 自陣の抜き出し(高さは自陣抜けられるくらい)
  // MoveToWaypoint,  // ウェイポイントへ移動（共通エリアの）(高さは共通ギリ)
  // MoveToCmnWait,  // 共通エリア待機状態へ移動(高さは共通ギリ)
  // MoveToCmnWork,  // 共通エリア上空へ移動（高さは共通ギリ）
  // CatchCmn,       // 共通の把持（高さは共通キャチ）
  // CmnAbove,  // ワークの抜き出し(高さは共通抜けられるくらい)
  // CmnStore,  // 共通のワークを自陣へリリース（高さは自陣キャチ）
  // MoveToShotBox,  // シューティングボックスへ移動(高さは共通ギリ)
  // MoveToRelease,  // シュートしに行く（高さは地面）
  // ShtTsukemen,    // つけ麺
  // MoveToBonus,    // ボーナスエリアへ行く（高さは地面）
  // Release,        // リリース（高さはワークに当たらない）
};
enum class Side { Blue, Red };
enum class Area { Init, Own, Cmn, Sht, Str };
enum class ZState {
  OwnGiri,
  OwnCatch,
  CmnGiri,
  CmnCatch,
  CmnAbove,
  ShtGiri,
  Shoot,
  ShtAbove,
  ShtTsuke,
};

struct LOCATION {
  float x;
  float y;
  float z;
};  // 位置

struct WORKLOCATION {
  std::pair<float, float> init_area;              // 初期位置
  std::vector<std::pair<float, float>> own_area;  // 自陣エリアのワークxy座標
  std::vector<std::pair<float, float>> cmn_area;  // 共有エリアのワークxy座標
  std::vector<std::pair<float, float>> sht_area;  // ボーナスエリアのxy座標
  std::vector<std::pair<float, float>> str_area;  // ストレージエリアのxy座標
  std::vector<float> stepper_state;  // 自陣エリアのワークz座標
};

class GeneralCommand {
 public:
  GeneralCommand(principal_interfaces::msg::Movecommand& _msg) : msg(_msg){};
  void init(Side _side, float _arm_offset, float _cmn_offset, float _sht_offset,
            float _hand_offset, float _finger_offset, WORKLOCATION _location);
  void move_to(Area area, int hand, int num, ZState z_state,
               bool is_cmn = false);
  void move_to(Area area, int hand, int num, bool is_cmn = false);
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
  float hand_offset;
  float finger_offset;
};

}  // namespace catch2023_principal