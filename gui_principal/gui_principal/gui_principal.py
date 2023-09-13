import rclpy
from rclpy.node import Node
import PySimpleGUI as sg
from principal_interfaces.msg import Parameters
from principal_interfaces.msg import Stateinfo
sg.theme('SystemDefaultForReal')


class GUIPrincipal(Node):
    def __init__(self):
        super().__init__('gui_principal')
        self.init_args()
        param_sub = self.create_subscription(Parameters, 'parameters', self.param_callback, 10)
        state_sub = self.create_subscription(Stateinfo, 'stateinfo', self.state_callback, 10)
        while not self.init_flag:
            pass
        self.make_window()

    def init_args(self):
        self.init_flag = False
        self.is_red = True
        self.state = {0: 'init', 1: 'move_to_own_work', 2: 'catch_own', 3: 'move_to_waypoint', 4: 'move_to_cmn_wait', 5: 'move_to_cmn_work',
                      6: 'catch_cmn', 7: 'cmn_above', 8: 'move_to_shotbox', 9: 'move_to_release', 10: 'move_to_bonus', 11: 'release'}
        self.state_name = self.state[0]

    def param_callback(self, msg):
        self.is_red = msg.isred
        self.init_flag = True

    def state_callback(self, msg):
        self.state_name = self.state[msg.statename]

    def make_window(self):
        cv = sg.Canvas(size=(400, 400), background_color='white')
        
        layout = [[sg.Text(' こ れ は P y S i m p l e G U I を 使 っ た サ ン プ ル プ ロ グ ラ ム で す ')],
                  [sg.Button('Quit '), sg.Button(' OK ')]]
        # ウィンドウ作成
        window = sg.Window('State Monitor Principal', layout, finalize=True)
        tcv = cv.TKCanvas
        tcv.draw_rectangle(0, 0, 100, 100, fill='red')
        # イベントループ
        while True:
            event, values = window.read()
            # イベントの読み取り（イベント待ち）
            self.get_logger().info('nyan')
            sg.theme('DarkAmber')
            # 確認表示
            if event in (None, ' Quit '):
                # 終 了 条 件 （ None : ク ロ ー ズ ボ タ ン ）
                self.get_logger().info('nyan2')
                break
        # 終了処理
        window . close()

        pass


def main(args=None):
    rclpy.init(args=args)
    guiprincipal = GUIPrincipal()
    rclpy.spin(guiprincipal)
    guiprincipal.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
