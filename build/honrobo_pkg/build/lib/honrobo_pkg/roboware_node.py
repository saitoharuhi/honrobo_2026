"""
roboware_node.py — ロボット制御統合ノード

PS4コントローラー(手動)とWebSocket(自動)の入力を統合し、
CAN送信指令を生成してcan_nodeに送信します。

サブスクライブ:
    /ps4_joy   (Joy)   — PS4入力
    /nav_cmd   (Twist) — 自動運転速度指令
    /auto_mode (Bool)  — 自動/手動モード切替

パブリッシュ:
    /can_tx (Int32MultiArray) — CAN送信データ
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray, Bool
from geometry_msgs.msg import Twist
import struct
import sys
import threading
from honrobo_pkg.shutdown_helper import ask_shutdown_action, trigger_stop_all

# ボタン表示名 (ps4_nodeのBUTTON_MAPと対応)
BUTTON_LABELS = [
    'Square', 'Cross', 'Circle', 'Triangle',
    'L1', 'R1', 'L2(Btn)', 'R2(Btn)',
    'SHARE', 'OPTIONS', 'PS', 'L3', 'R3',
    'UP', 'DOWN', 'LEFT', 'RIGHT',
]
AXIS_LABELS = ['LX', 'LY', 'RX', 'RY', 'L2', 'R2']

MAX_SPEED = 500.0       # 最大並進速度 (mm/s)
MAX_ANGULAR = 45.0      # 最大回転速度 (deg/s)
VEL_SCALE = 10.0        # CAN送信時のスケール倍率


class RobowareNode(Node):
    def __init__(self):
        super().__init__('roboware_node')

        # サブスクライバー
        self.create_subscription(Joy, 'ps4_joy', self._joy_cb, 10)
        self.create_subscription(Bool, 'auto_mode', self._mode_cb, 10)
        self.create_subscription(Twist, 'nav_cmd', self._nav_cb, 10)

        # パブリッシャー
        self.can_pub = self.create_publisher(Int32MultiArray, 'can_tx', 10)

        self.auto_mode = False
        self.state = {
            'mode': 'MANUAL', 'axes': [0.0] * 6,
            'buttons': [], 'nav_cmd': 'None', 'last_can': 'None',
        }
        self.lock = threading.Lock()
        self.create_timer(0.05, self._print_display)

    def _mode_cb(self, msg):
        self.auto_mode = msg.data
        with self.lock:
            self.state['mode'] = (
                'AUTO [STICKS LOCKED]' if msg.data else 'MANUAL'
            )

    def _nav_cb(self, msg):
        with self.lock:
            self.state['nav_cmd'] = (
                f"X:{msg.linear.x:>4.0f} Y:{msg.linear.y:>4.0f} "
                f"Z:{msg.angular.z:>4.0f}"
            )
        if self.auto_mode:
            vx = int(msg.linear.x * VEL_SCALE)
            vy = int(msg.linear.y * VEL_SCALE)
            vz = int(msg.angular.z * VEL_SCALE)
            data = struct.pack('>hhh', vx, vy, vz)
            self._send_can(0x510, data)

    def _joy_cb(self, msg):
        with self.lock:
            self.state['axes'] = list(msg.axes)
            self.state['buttons'] = [
                BUTTON_LABELS[i]
                for i, v in enumerate(msg.buttons)
                if i < len(BUTTON_LABELS) and v == 1
            ]

        # 手動モード時のスティック→CAN送信
        if not self.auto_mode:
            vx = int((msg.axes[0] * MAX_SPEED) * VEL_SCALE)
            vy = int((msg.axes[1] * MAX_SPEED) * VEL_SCALE)
            vz = int((msg.axes[2] * MAX_ANGULAR) * VEL_SCALE)
            data = struct.pack('>hhh', vx, vy, vz)
            self._send_can(0x510, data)

        # ボタン情報のCAN送信
        if len(msg.buttons) > 16:
            # 0x500: ○△×□ + 矢印
            b500 = [
                msg.buttons[2], msg.buttons[3],
                msg.buttons[1], msg.buttons[0],
                msg.buttons[13], msg.buttons[14],
                msg.buttons[15], msg.buttons[16],
            ]
            self._send_can(0x500, b500)

            # 0x501: R1,R2,R3,L1,L2,L3
            b501 = [
                msg.buttons[5], msg.buttons[7], msg.buttons[12],
                msg.buttons[4], msg.buttons[6], msg.buttons[11],
                0, 0,
            ]
            self._send_can(0x501, b501)

            # 0x502: Share, Options, PS
            b502 = [
                msg.buttons[8], msg.buttons[9], msg.buttons[10],
                0, 0, 0, 0, 0,
            ]
            self._send_can(0x502, b502)

    def _send_can(self, can_id, data):
        """CAN送信データをcan_nodeへパブリッシュ"""
        msg = Int32MultiArray()
        msg.data = [can_id] + list(data)
        self.can_pub.publish(msg)
        with self.lock:
            self.state['last_can'] = (
                f"ID:0x{can_id:03X} Data:{list(data)}"
            )

    def _print_display(self):
        with self.lock:
            sys.stdout.write('\033[2J\033[H')
            sys.stdout.write("=" * 52 + "\n")
            sys.stdout.write(
                f" ROBOWARE NODE | Mode: {self.state['mode']}\n"
            )
            sys.stdout.write("=" * 52 + "\n")

            sys.stdout.write("[STICKS]\n")
            for i, lbl in enumerate(AXIS_LABELS):
                val = (self.state['axes'][i]
                       if i < len(self.state['axes']) else 0.0)
                if lbl in ('L2', 'R2'):
                    sys.stdout.write(f"  {lbl}: {val:5.2f} |")
                else:
                    spd = val * MAX_SPEED
                    sys.stdout.write(f"  {lbl}: {spd:>6.1f} mm/s |")
                if i % 2 == 1:
                    sys.stdout.write("\n")

            btns = (", ".join(self.state['buttons'])
                    if self.state['buttons'] else "None")
            sys.stdout.write(f"\n[BUTTONS] {btns}\n")
            sys.stdout.write(f"[NAV CMD] {self.state['nav_cmd']}\n")
            sys.stdout.write(f"[CAN TX]  {self.state['last_can']}\n")
            sys.stdout.write("=" * 52 + "\n")
            sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = RobowareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        choice = ask_shutdown_action('roboware_node')
        if choice == 'a':
            trigger_stop_all()
        # 'y' と 'c' の場合はそのまま終了またはループ継続
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
