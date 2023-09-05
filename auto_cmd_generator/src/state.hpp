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

class GeneralCommand {
 public:
  GeneralCommand();
  void init(principal_interfaces::msg::Movecommand &msg, Side side);
  void move_to(float x, float y, float z, Area area);
  void grasp(int n);
  void release();

 private:
  principal_interfaces::msg::Movecommand msg;
  Side side;
  bool is_init = false;
};

}  // namespace catch2023_principal