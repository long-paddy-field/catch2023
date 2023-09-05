import rclpy  # ROS2のPythonモジュール
from rclpy.node import Node
from std_msgs.msg import Bool  # トピック通信に使うStringメッセージ型をインポート
from principal_interfaces.msg import Statecommand, Movecommand
import csv
from state import State, init_state


class AutoCmdGen(Node):
    field_index = 0
    shoot_index = 0
    hold_count = 0
    state = State.Init
    prev_cmd = Movecommand()

    def __init__(self):
        super().__init__('auto_cmd_gen')

        self.cmd = Movecommand()
        self.timer = self.create_timer(0.1, self.timer_callback)  # timerの宣言
        self.state_command_sub = self.create_subscription(
            Statecommand, '/state_command', self.state_command_callback, 10)
        self.is_auto_sub = self.create_subscription(Bool, '/is_auto',
                                                    self.is_auto_callback, 10)
        self.current_pos_sub = self.create_subscription(
            Movecommand, '/current_pos', self.current_pos_callback, 10)
        self.auto_command_pub = self.create_publisher(Movecommand,
                                                      '/auto_move_command', 10)

        self.declare_parameter('side', 'blue')
        self.declare_parameter('config_folder', __file__ + '../config')

        self.side = self.get_parameter(
            'side').get_parameter_value().string_value

        # targetのcsvを読み込む
        target_csv_path = self.get_parameter(
            'config_folder').get_parameter_value(
        ).string_value + '/' + self.side + '.csv'
        with open(target_csv_path) as f:
            reader = csv.reader(f)
            self.target_pos = [{
                "x": row[0],
                "y": row[1],
                "is_common": not not row[2]
            } for row in reader[1:]]

        # シューティングエリアのcsvを読み込む
        shoot_csv_path = self.get_parameter(
            'config_folder').get_parameter_value(
        ).string_value + '/' + self.side + '_shoot.csv'
        with open(shoot_csv_path) as f:
            reader = csv.reader(f)
            self.shoot_pos = [{
                "x": row[0],
                "y": row[1],
            } for row in reader[1:]]

        self.transition(State.Init)

    def transition(self, state: State):
        field_target = self.target_pos[self.field_index]
        shoot_target = self.shoot_pos[self.shoot_index]

        match state:
            case State.Init:
                init_state(self)
                cmd.x = 0
                cmd.y = 0 if self.side == 'blue' else 180
                cmd.z = 0
                cmd.rotate = 1 if self.side == 'blue' else -1
                cmd.hand = [0, 0, 0]
                self.auto_command_pub.publish(cmd)
                self.prev_cmd = cmd

            case State.CatchMove:
                cmd.x = field_target["x"]
                cmd.y = field_target["y"]
                cmd.z = 0
                if field_target["is_common"]:
                    cmd.rotate = 0
                elif self.side == 'blue':
                    cmd.rotate = -1
                else:
                    cmd.rotate = 1
                cmd.hand = self.prev_cmd.hand
                self.auto_command_pub.publish(cmd)
                self.prev_cmd = cmd

            case State.CatchDown:
                hand = [0, 0, 0]
                self.hold_count += 1
                hand[self.hold_count - 1] = 1

                cmd.x = self.prev_cmd.x
                cmd.y = self.prev_cmd.y
                cmd.z = 1 if field_target["is_common"] else 2
                cmd.rotate = self.prev_cmd.rotate
                cmd.hand = hand
                self.auto_command_pub.publish(cmd)
                self.prev_cmd = cmd

            case State.CatchHold:
                cmd.x = self.prev_cmd.x
                cmd.y = self.prev_cmd.y
                cmd.z = self.prev_cmd.z
                cmd.rotate = self.prev_cmd.rotate
                cmd.hand = [0, 0, 0]
                self.auto_command_pub.publish(cmd)
                self.prev_cmd = cmd
                pass
            case State.CatchUp:
                cmd.x = self.prev_cmd.x
                cmd.y = self.prev_cmd.y

                if self.hold_count == 3:
                    cmd.z = 0
                elif field_target["is_common"]:
                    cmd.z = 0
                else:
                    cmd.z = 1

                cmd.rotate = self.prev_cmd.rotate
                cmd.hand = self.prev_cmd.hand
                self.auto_command_pub.publish(cmd)
                self.prev_cmd = cmd

            case State.ShootMove:
                cmd.x = shoot_target["x"] + 100  # シュート時のスライド量
                cmd.y = shoot_target["y"]
                cmd.z = 0
                cmd.rotate = 1 if self.side == 'blue' else -1
                cmd.hand = [0, 0, 0]
                self.auto_command_pub.publish(cmd)
                self.prev_cmd = cmd

            case State.ShootDown:
                cmd.x = self.prev_cmd.x
                cmd.y = self.prev_cmd.y
                cmd.z = 1
                cmd.rotate = self.prev_cmd.rotate
                cmd.hand = self.prev_cmd.hand
                self.auto_command_pub.publish(cmd)
                self.prev_cmd = cmd

            case State.ShootSlide:
                cmd.x = shoot_target['x']
                cmd.y = self.prev_cmd.y
                cmd.z = self.prev_cmd.z
                cmd.rotate = self.prev_cmd.rotate
                cmd.hand = self.prev_cmd.hand
                self.auto_command_pub.publish(cmd)
                self.prev_cmd = cmd

            case State.ShootRelease:
                cmd.x = self.prev_cmd.x
                cmd.y = self.prev_cmd.y
                cmd.z = self.prev_cmd.z
                cmd.rotate = self.prev_cmd.rotate
                cmd.hand = [1, 1, 1]
                self.auto_command_pub.publish(cmd)
                self.prev_cmd = cmd

            case State.ShootUp:
                cmd.x = self.prev_cmd.x
                cmd.y = self.prev_cmd.y
                cmd.z = 0
                cmd.rotate = self.prev_cmd.rotate
                cmd.hand = [0, 0, 0]
                self.auto_command_pub.publish(cmd)
                self.prev_cmd = cmd

        self.state = state

    def timer_callback(self):
        match self.state:
            case State.Init:
                pass
            case State.CatchMove:
                pass
            case State.CatchDown:
                pass
            case State.CatchHold:
                pass
            case State.CatchUp:
                pass
            case State.ShootDown:
                pass
            case State.ShootMove:
                pass
            case State.ShootRelease:
                pass
            case State.ShootUp:
                pass

    def state_command_callback(self, msg):
        pass

    def is_auto_callback(self, msg):
        pass

    def current_pos_callback(self, msg):
        pass

    def send_command(self):
        self.auto_command_pub.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AutoCmdGen()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
