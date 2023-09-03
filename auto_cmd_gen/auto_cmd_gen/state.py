from enum import Enum


class State(Enum):
    Init = 0,
    CatchMove = 1,
    CatchDown = 2,
    CatchHold = 3,
    CatchUp = 4,
    ShootMove = 5,
    ShootDown = 6,
    ShootSlide = 7,
    ShootRelease = 8,
    ShootUp = 8,
