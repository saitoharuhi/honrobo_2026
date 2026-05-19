import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import struct
import sys
import threading
import time

# ==========================================
# 表示設定 (ps4_nodeのBUTTON_MAPと一致させてください)
# ==========================================
BUTTON_LABELS = [
    '□ (Square)',
    '× (Cross)',
    '○ (Circle)',
    '△ (Triangle)',
    'L1',
    'R1',
    'L2 (Button)',
    'R2 (Button)',
    'SHARE',
    'OPTIONS',
    'PS',
    'L3',
    'R3',
    'UP',
    'DOWN',
    'LEFT',
    'RIGHT'
]

# スティックの表示名
AXIS_LABELS = ['LX', 'LY', 'RX', 'RY', 'L2', 'R2']

# 速度の最大値 (mm/s)
MAX_SPEED = 500.0
# ==========================================

class RobowareNode(Node):
    def __init__(self):
        super().__init__('roboware_node')
        
        # サブスクリプション設定
        self.subscription = self.create_subscription(Joy, 'ps4_joy', self.listener_callback, 10)
        self.mode_sub = self.create_subscription(Bool, 'auto_mode', self.mode_callback, 10)
        self.nav_sub = self.create_subscription(Twist, 'nav_cmd', self.nav_callback, 10)
        
        # パブリッシャー設定
        self.can_pub = self.create_publisher(Int32MultiArray, 'can_tx_topic', 10)

        # 内部状態
        self.auto_mode = False
        
        # 表示用データ
        self.state = {
            'mode': 'MANUAL',
            'axes': [0.0] * 6,
            'buttons': [],
            'nav_cmd': 'None',
            'last_can': 'None'
        }
        self.display_lock = threading.Lock()
        self.timer = self.create_timer(0.05, self.print_display) # 20Hz更新

    def mode_callback(self, msg):
        self.auto_mode = msg.data
        with self.display_lock:
            self.state['mode'] = 'AUTO DRIVING [STICKS LOCKED]' if msg.data else 'MANUAL'

    def nav_callback(self, msg):
        with self.display_lock:
            self.state['nav_cmd'] = f"X:{msg.linear.x:>4.0f}, Y:{msg.linear.y:>4.0f}, Z:{msg.angular.z:>4.0f}"
        
        if self.auto_mode:
            # X(前後), Y(左右), Z(回転)
            scale = 10.0
            # デモコードに合わせて LX(左右)=vx, LY(上下)=vy, RX(回転)=vz に対応させる
            # Twistメッセージの扱いは一旦そのままにし、スケールのみ調整
            vx = int(msg.linear.x * scale)
            vy = int(msg.linear.y * scale)
            vz = int(msg.angular.z * scale)
            
            # デモスクリプトに合わせてビッグエンディアン(>hhh)でパッキング
            stick_bytes = struct.pack('>hhh', vx, vy, vz)
            self.send_can(0x510, stick_bytes)

    def send_can(self, can_id, data_bytes):
        msg = Int32MultiArray()
        msg.data = [can_id] + list(data_bytes)
        self.can_pub.publish(msg)
        
        with self.display_lock:
            self.state['last_can'] = f"ID:0x{can_id:03x} Data:{list(data_bytes)}"

    def listener_callback(self, msg):
        with self.display_lock:
            self.state['axes'] = list(msg.axes)
            active = []
            for i, val in enumerate(msg.buttons):
                if i < len(BUTTON_LABELS) and val == 1:
                    active.append(BUTTON_LABELS[i])
            self.state['buttons'] = active

        # CAN送信処理 (0x510)
        if not self.auto_mode:
            # デモコードに完全一致させる: vx=LX, vy=LY, vz=RX
            # 回転の最大値(MAX_ANGULAR)は 45.0 に設定
            scale = 10.0
            MAX_ANGULAR = 45.0
            vx = int((msg.axes[0] * MAX_SPEED) * scale) # LX
            vy = int((msg.axes[1] * MAX_SPEED) * scale) # LY
            vz = int((msg.axes[2] * MAX_ANGULAR) * scale) # RX
            stick_bytes = struct.pack('>hhh', vx, vy, vz)
            self.send_can(0x510, stick_bytes)

        # デモスクリプトに合わせて、ビット演算ではなくバイト配列(0 or 1)として送信
        if len(msg.buttons) > 16:
            # --- 0x500 (矢印ボタンと○、×、△、□) ---
            # 順番: ○, △, ×, □, UP, DOWN, LEFT, RIGHT
            b500 = [
                msg.buttons[2], msg.buttons[3], msg.buttons[1], msg.buttons[0],
                msg.buttons[13], msg.buttons[14], msg.buttons[15], msg.buttons[16]
            ]
            self.send_can(0x500, b500)

            # --- 0x501 (R1, R2, R3, L1, L2, L3) ---
            b501 = [
                msg.buttons[5], msg.buttons[7], msg.buttons[12],
                msg.buttons[4], msg.buttons[6], msg.buttons[11],
                0, 0
            ]
            self.send_can(0x501, b501)

            # --- 0x502 (PS, SHARE, OPTIONS) ---
            # デモコードに完全一致させる: PS(10), SHARE(8), OPTIONS(9)
            b502 = [
                msg.buttons[10], msg.buttons[8], msg.buttons[9],
                0, 0, 0, 0, 0
            ]
            self.send_can(0x502, b502)

    def print_display(self):
        with self.display_lock:
            # 画面クリアとカーソルホーム
            sys.stdout.write('\033[2J\033[H')
            
            sys.stdout.write("====================================================\n")
            sys.stdout.write(f" ROBOWARE CONTROL NODE | Mode: {self.state['mode']}\n")
            sys.stdout.write("====================================================\n")
            
            # スティック表示
            sys.stdout.write("[PS4 STICKS]\n")
            for i, label in enumerate(AXIS_LABELS):
                val = self.state['axes'][i] if i < len(self.state['axes']) else 0.0
                if label in ['L2', 'R2']:
                    sys.stdout.write(f"  {label}: {val:5.2f} |")
                else:
                    speed = val * MAX_SPEED
                    sys.stdout.write(f"  {label}: {speed:>6.1f} mm/s |")
                if i % 2 == 1: sys.stdout.write("\n")
            
            # ボタン表示
            btns = ", ".join(self.state['buttons']) if self.state['buttons'] else "None"
            sys.stdout.write(f"\n[PS4 BUTTONS] {btns}\n")
            
            # 自動運転指令
            sys.stdout.write(f"\n[AUTO NAV CMD] {self.state['nav_cmd']}\n")
            
            # 最新のCAN送信
            sys.stdout.write(f"\n[LAST CAN TX]  {self.state['last_can']}\n")
            sys.stdout.write("====================================================\n")
            sys.stdout.flush()



def main(args=None):
    rclpy.init(args=args)
    node = RobowareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
