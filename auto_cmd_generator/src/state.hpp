#pragma once
#include "principal_interfaces/msg/movecommand.hpp"
namespace catch2023_principal {
enum class StateName {
  Init,
  MoveToOwnWork,
  MoveToCmnWork,
  CatchOwn,
  CatchCmn,
  MoveToShoot,
  Release,
};
enum class Side { Blue, Red };
enum class Area { Own, Cmn, Shb };

typedef struct LOCATION {
  float x;
  float y;
  float z;
};  // 位置

class GeneralCommand {
 public:
  GeneralCommand();
  void init(principal_interfaces::msg::Movecommand &msg, Side side,
            float arm_offset);
  void move_to(LOCATION location, Area area, int hand);
  void grasp(int n);
  void release();

 private:
  principal_interfaces::msg::Movecommand msg;
  Side side;
  bool is_init = false;
  float arm_offset;
};

}  // namespace catch2023_principal