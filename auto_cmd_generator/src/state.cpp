#include "state.hpp"
using namespace catch2023_principal;

void GeneralCommand::init(Side _side, float _arm_offset, float _cmn_offset,
                          float _sht_offset, WORKLOCATION _location) {
  side = _side;
  arm_offset = _arm_offset;
  cmn_offset = _cmn_offset;
  sht_offset = _sht_offset;
  location = _location;
  msg.x = 0;
  msg.y = 0;
  msg.z = 0;
  msg.rotate = 0;
  msg.hand[0] = false;
  msg.hand[1] = false;
  msg.hand[2] = false;
}

void GeneralCommand::move_to(Area area, int hand, int num, ZState z_state,
                             bool is_advance) {
  switch (area) {
    case Area::Cmn:
      if (side == Side::Red) {
        msg.x = location.cmn_area[num].first + arm_offset * (1 - hand);
        msg.y = location.cmn_area[num].second + (is_advance ? cmn_offset : 0);
        msg.z = location.stepper_state[static_cast<int>(z_state)];
        msg.rotate = 1;
      } else if (side == Side::Blue) {
        msg.x = location.cmn_area[num].first + arm_offset * (hand - 1);
        msg.y = location.cmn_area[num].second + (is_advance ? cmn_offset : 0);
        msg.z = location.stepper_state[static_cast<int>(z_state)];
        msg.rotate = -1;
      }
      break;
    case Area::Own:
      msg.x = location.own_area[num].first;
      msg.y = location.own_area[num].second + arm_offset * (1 - hand);
      msg.z = location.stepper_state[static_cast<int>(z_state)];
      msg.rotate = 0;
      break;
    case Area::Sht:
      msg.x = location.sht_area[num].first + (is_advance ? 0 : sht_offset);
      msg.y = location.sht_area[num].second;
      msg.z = location.stepper_state[static_cast<int>(z_state)];
      msg.rotate = 0;
      break;
    default:
      break;
  }
}

void GeneralCommand::move_to(ZState z_state) {
  msg.z = location.stepper_state[static_cast<int>(z_state)];
}
void GeneralCommand::grasp(Area area, int n) {
  if (area == Area::Cmn) {
    if (side == Side::Blue) {
      msg.hand[n] = true;
    } else if (side == Side::Red) {
      msg.hand[2 - n] = true;
    }
  } else {
    msg.hand[n] = true;
  }
}

void GeneralCommand::release() {
  msg.hand[0] = false;
  msg.hand[1] = false;
  msg.hand[2] = false;
}