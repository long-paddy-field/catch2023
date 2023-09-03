#pragma once

enum class State {
  Init,
  CatchMove,
  CatchDown,
  CatchHold,
  CatchUp,
  ShootDown,
  ShootMove,
  ShootRelease,
  ShootUp,
};

enum class Side { Blue, Red };