#include "state.hpp"
using namespace catch2023_principal;

void GeneralCommand::init(principal_interfaces::msg::Movecommand &_msg,
                          Side _side) {
  msg = _msg;
  side = _side;
  msg.x = 0;
  msg.y = 0;
  msg.z = 0;
  msg.rotate = 0;
  msg.hand[0] = false;
  msg.hand[1] = false;
  msg.hand[2] = false;
}

void GeneralCommand::move_to(float x, float y, float z, Area area) {
  msg.x = x;
  msg.y = y;
  msg.z = z;
  if (area == Area::Cmn) {
    msg.rotate = 0;
  } else {
    msg.rotate = (side == Side::Blue) ? 1 : -1;
  }
}

void GeneralCommand::grasp(int n) {
  if (side == Side::Blue) {
    msg.hand[n] = true;
  } else if (side == Side::Red) {
    msg.hand[2 - n] = true;
  }
}

void GeneralCommand::release() {
  msg.hand[0] = false;
  msg.hand[1] = false;
  msg.hand[2] = false;
}