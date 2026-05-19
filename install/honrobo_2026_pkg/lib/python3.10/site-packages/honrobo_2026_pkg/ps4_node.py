import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame
import sys

# ==========================================
# コントローラーの設定 (必要に応じて変更してください)
# ==========================================
# スティックの反転 (1: そのまま, -1: 反転)
AXIS_INVERT = {
    'LX': 1,   # 左スティック左右
    'LY': -1,  # 左スティック上下 (通常、上はマイナスなので反転させる)
    'RX': 1,   # 右スティック左右
    'RY': -1,  # 右スティック上下
    'L2': 1,   # L2トリガー
    'R2': 1    # R2トリガー
}

# ボタンのインデックス設定 (pygameの検出順序に合わせる)
# 環境によって番号が違う場合、ここを書き換えてください
BUTTON_MAP = {
    'SQUARE': 2,   # 変更: 実際のPS4では 2 が □
    'CROSS': 0,    # 変更: 実際のPS4では 0 が ×
    'CIRCLE': 1,   # 変更: 実際のPS4では 1 が ○
    'TRIANGLE': 3, # 変更: 実際のPS4では 3 が △
    'L1': 4,
    'R1': 5,
    'L2_BTN': 6,
    'R2_BTN': 7,
    'SHARE': 8,
    'OPTIONS': 9,
    'PS': 10,
    'L3': 11,
    'R3': 12,
    'UP': 13,
    'DOWN': 14,
    'LEFT': 15,
    'RIGHT': 16
}

# スティックのインデックス設定
AXIS_MAP = {
    'LX': 0,
    'LY': 1,
    'RX': 3,
    'RY': 4,
    'L2': 2,
    'R2': 5
}
# ==========================================

class PS4Node(Node):
    def __init__(self):
        super().__init__('ps4_node')
        self.publisher_ = self.create_publisher(Joy, 'ps4_joy', 10)
        
        # Pygameの初期化
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            self.get_logger().error('PS4コントローラーが見つかりません！')
            sys.exit()
            
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f'コントローラーを認識しました: {self.joystick.get_name()}')
        
        # 定期的に値を読み取るタイマー (50Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        pygame.event.pump()
        
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # スティックの値を取得 (-1.0 ～ 1.0)
        # 軸の数はコントローラーに依存するため、安全に取得
        num_axes = self.joystick.get_numaxes()
        axes = [0.0] * 6 # LX, LY, L2, RX, RY, R2 の順を想定
        
        def get_axis_val(name):
            idx = AXIS_MAP.get(name)
            if idx is not None and idx < num_axes:
                val = self.joystick.get_axis(idx) * AXIS_INVERT.get(name, 1)
                # L2, R2の処理: -1.0～1.0を0.0～1.0に変換
                if name in ['L2', 'R2']:
                    return (val + 1.0) / 2.0
                
                # スティックの遊び（デッドゾーン）を設定し、微小なずれを0.0にする
                deadzone = 0.05
                if abs(val) < deadzone:
                    return 0.0
                return val
            return 0.0

        msg.axes = [
            get_axis_val('LX'),
            get_axis_val('LY'),
            get_axis_val('RX'),
            get_axis_val('RY'),
            get_axis_val('L2'),
            get_axis_val('R2')
        ]
        
        # ボタンの値を取得 (0 または 1)
        num_buttons = self.joystick.get_numbuttons()
        # 全てのボタン状態を取得するリスト (BUTTON_MAPの順番通り)
        buttons_state = [0] * len(BUTTON_MAP)
        
        for name, idx in BUTTON_MAP.items():
            if idx < num_buttons:
                # ボタンが押されていれば 1, そうでなければ 0
                buttons_state[list(BUTTON_MAP.keys()).index(name)] = self.joystick.get_button(idx)
        
        # 十字キー（Hat）の取得
        num_hats = self.joystick.get_numhats()
        if num_hats > 0:
            hat = self.joystick.get_hat(0) # (x, y)
            # UP: y=1, DOWN: y=-1, LEFT: x=-1, RIGHT: x=1
            buttons_state[list(BUTTON_MAP.keys()).index('UP')] = 1 if hat[1] == 1 else 0
            buttons_state[list(BUTTON_MAP.keys()).index('DOWN')] = 1 if hat[1] == -1 else 0
            buttons_state[list(BUTTON_MAP.keys()).index('LEFT')] = 1 if hat[0] == -1 else 0
            buttons_state[list(BUTTON_MAP.keys()).index('RIGHT')] = 1 if hat[0] == 1 else 0

        msg.buttons = buttons_state
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PS4Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
