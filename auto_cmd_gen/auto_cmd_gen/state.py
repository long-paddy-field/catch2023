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


def init_state(self):
    self.cmd.x = 0
    self.cmd.y = -0.820 if self.side == "blue" else 0.820
    self.cmd.z = 0
    self.cmd.rotate = -1 if self.side == "blue" else 1
    self.cmd.hand = [False, False, False]


def catch_move(self):
    if self.shot_count == 0:
        # 最初の1つ目は1個だけでシュート
        pass
    elif 0 < self.shot_count and self.shot_count+self.shift_count < 10:
        # １個目から９個目のワークは共通エリアに行く
        pass
    else:
        # あとは自陣エリアでがんばる
        pass
    self.cmd.x = 0
    self.cmd.y = -0.820 if self.side == "blue" else 0.820
    self.cmd.z = 0
    self.cmd.rotate = -1 if self.side == "blue" else 1
    self.cmd.hand = [False, False, False]
