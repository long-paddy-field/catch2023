#pragma once

#include "rclcpp/rclcpp.hpp"

namespace catch2023_principal {
enum class BUTTONS {
  X = 0,
  A = 1,
  B = 2,
  Y = 3,
  LB = 4,
  RB = 5,
  LT = 6,
  RT = 7,
  BACK = 8,
  START = 9,
  LS = 10,
  RS = 11,
  LC = 12,
  RC = 13,
  UC = 14,
  DC = 15,
};
enum class BUTTON_TYPE {
  ON_OFF = 0,        // 押すたびにON(TRUE)-OFF(FALSE)切り替え
  PUSH_RELEASE = 1,  // 押しているとON(TRUE),離すとOFF(FALSE）
  PULSER = 2,        // 押したら1回だけパルス出る
};
class ButtonManager {
 public:
  // constructor
  ButtonManager(){};
  ButtonManager(BUTTON_TYPE _button_type);
  // destructor
  ~ButtonManager(){};
  // methods
  void set(bool btn);
  bool read();
  operator bool() { return read(); };

 private:
  // member variables
  BUTTON_TYPE button_type;
  rclcpp::Node *node;
  bool on_off = false;
  bool push_release = false;
  bool pulser = false;
  double time_counter = 0;
  bool past_btn = false;
};
};  // namespace catch2023_principal