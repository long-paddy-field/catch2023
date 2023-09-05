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
        match state:
            case State.Init:

            case State.CatchMove:

            case State.CatchDown:

            case State.CatchHold:
                pass
            case State.CatchUp:

            case State.ShootMove:
            case State.ShootDown:

            case State.ShootSlide:

            case State.ShootRelease:

            case State.ShootUp:



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
