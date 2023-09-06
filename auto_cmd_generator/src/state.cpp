#include "state.hpp"
using namespace catch2023_principal;

void GeneralCommand::init(principal_interfaces::msg::Movecommand &_msg,
                          Side _side, float _arm_offset) {
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

void GeneralCommand::move_to(LOCATION location, Area area, int hand) {
  if (area == Area::Cmn) {
    if (side == Side::Red) {
      msg.x = location.x + arm_offset * (1 - hand);
      msg.y = location.y;
      msg.z = location.z;
      msg.rotate = 1;
    } else if (side == Side::Blue) {
      msg.x = location.x + arm_offset * (hand - 1);
      msg.y = location.y;
      msg.z = location.z;
      msg.rotate = -1;
    }
  } else {
    msg.x = location.x;
    msg.y = location.y + arm_offset * (1 - hand);
    msg.z = location.z;
    msg.rotate = 0;
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