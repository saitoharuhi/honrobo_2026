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
from nav_msgs.msg import Odometry
import struct
import sys
import threading
import math
import time

# ボタン表示名 (ps4_nodeのBUTTON_MAPと対応)
BUTTON_LABELS = [
    'Square', 'Cross', 'Circle', 'Triangle',
    'L1', 'R1', 'L2(Btn)', 'R2(Btn)',
    'SHARE', 'OPTIONS', 'PS', 'L3', 'R3',
    'UP', 'DOWN', 'LEFT', 'RIGHT',
]
AXIS_LABELS = ['LX', 'LY', 'RX', 'RY', 'L2', 'R2']

MAX_SPEED = 2000.0       # 最大並進速度 (mm/s)
MAX_ANGULAR = 90.0      # 最大回転速度 (deg/s)
VEL_SCALE = 10.0        # CAN送信時のスケール倍率


class RobowareNode(Node):
    def __init__(self):
        super().__init__('roboware_node')

        # サブスクライバー
        self.create_subscription(Joy, 'ps4_joy', self._joy_cb, 10)
        self.create_subscription(Bool, 'auto_mode', self._mode_cb, 10)
        self.create_subscription(Twist, 'nav_cmd', self._nav_cb, 10)
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)

        # 姿勢(Yaw角)
        self.current_yaw = 0.0
        self.odom_count = 0

        # パブリッシャー
        self.can_pub = self.create_publisher(Int32MultiArray, 'can_tx', 10)
        self.mode_pub = self.create_publisher(Bool, 'auto_mode', 10)

        self.auto_mode = False
        self.latest_joy_msg = None
        self.latest_nav_msg = None
        self.state = {
            'mode': 'MANUAL', 'axes': [0.0] * 6,
            'buttons': [], 'nav_cmd': 'None', 'last_can': 'None',
        }
        self.lock = threading.Lock()
        self.control_style = "LOCAL"
        self.field_oriented_mode = False  # モードのトグル状態 (True: FIELD / False: LOCAL)
        self.prev_triangle_state = 0      # 三角ボタンの前回の状態
        self.create_timer(0.05, self._print_display)
        # CAN送信周波数を 100Hz (0.01秒周期 / 10ms) に統一するタイマー
        self.create_timer(0.01, self._can_tx_timer)

    def _odom_cb(self, msg):
        """自己位置オドメトリから現在の姿勢(Yaw)をラジアンで取得し、CAN送信(0x520)"""
        self.odom_count += 1
        q = msg.pose.pose.orientation
        # クォータニオンからYaw(ヨー角)への変換
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # 0x520: 自己位置のYaw (度数法の整数, int16型2B, ビッグエンディアン) を送信
        try:
            yaw_deg = math.degrees(self.current_yaw)
            yaw_int = int(round(yaw_deg))
            data = struct.pack('>h', yaw_int)
            self._send_can(0x520, data)
        except Exception as e:
            self.get_logger().error(f"Failed to send Yaw via CAN 0x520: {e}")

    def _mode_cb(self, msg):
        self.auto_mode = msg.data
        with self.lock:
            self.state['mode'] = (
                'AUTO [STICKS LOCKED]' if msg.data else 'MANUAL'
            )

    def _nav_cb(self, msg):
        vx_display = msg.linear.x * 1000.0
        vy_display = msg.linear.y * 1000.0
        vz_display = math.degrees(msg.angular.z)
        with self.lock:
            self.state['nav_cmd'] = (
                f"X(Lat):{vx_display:>4.0f} Y(Fwd):{vy_display:>4.0f} "
                f"Z:{vz_display:>4.0f}"
            )
        self.latest_nav_msg = msg

    def _joy_cb(self, msg):
        # 1. 画面表示用の状態更新（表示のみ）
        with self.lock:
            self.state['axes'] = list(msg.axes)
            self.state['buttons'] = [
                BUTTON_LABELS[i]
                for i, v in enumerate(msg.buttons)
                if i < len(BUTTON_LABELS) and v == 1
            ]
        self.latest_joy_msg = msg

        # 三角ボタン (buttons[3]) の立ち上がりエッジ検出で FIELD/LOCAL 切り替え
        if len(msg.buttons) > 3:
            current_triangle = msg.buttons[3]
            if current_triangle == 1 and self.prev_triangle_state == 0:
                self.field_oriented_mode = not self.field_oriented_mode
                self.get_logger().info(f"操縦モード切替: {'FIELD (マップ基準)' if self.field_oriented_mode else 'LOCAL (ロボット基準)'}")
            self.prev_triangle_state = current_triangle

        # 2. 自動運転中の緊急割り込み（コントローラー操作を検知したら自動運転を即座に非常停止）
        if self.auto_mode:
            joy_active = False
            # 手動操作で使用している軸 (0: LX, 1: LY, 2: RX) の入力チェック
            for idx in [0, 1, 2]:
                if idx < len(msg.axes) and abs(msg.axes[idx]) > 0.5:
                    joy_active = True
                    break
            # いずれかのボタンが押された場合もチェック
            for btn in msg.buttons:
                if btn == 1:
                    joy_active = True
                    break

            if joy_active:
                self.get_logger().warn("PS4コントローラー操作を検知: 自動運転を緊急停止し、手動モードへ切り替えます。")
                # 1. 自動運転モードを解除
                self.auto_mode = False
                mode_msg = Bool()
                mode_msg.data = False
                self.mode_pub.publish(mode_msg)

                # 2. ロボットを即座に非常停止 (速度 0)
                data = struct.pack('>hhh', 0, 0, 0)
                self._send_can(0x510, data)

                with self.lock:
                    self.state['mode'] = 'MANUAL (EMERGENCY STOP)'

    def _can_tx_timer(self):
        """1000Hz (0.001秒周期 / 1ms) でCANデータを定周期パブリッシュ"""
        if self.auto_mode:
            if self.latest_nav_msg is not None:
                msg = self.latest_nav_msg
                vx = int(msg.linear.x * 1000.0 * VEL_SCALE)
                vy = int(msg.linear.y * 1000.0 * VEL_SCALE)
                vz = int(math.degrees(msg.angular.z) * VEL_SCALE)
                data = struct.pack('>hhh', vx, vy, vz)
                self._send_can(0x510, data)
        else:
            if self.latest_joy_msg is not None:
                msg = self.latest_joy_msg
                # 手動モード時のスティック→CAN送信 (0x510)
                v_x_field = -msg.axes[0] * MAX_SPEED
                v_y_field = msg.axes[1] * MAX_SPEED
                vz = int((msg.axes[2] * MAX_ANGULAR) * VEL_SCALE)

                # 三角ボタンで切り替えたモード状態を使用する
                is_field_oriented = self.field_oriented_mode
                self.control_style = "FIELD" if is_field_oriented else "LOCAL"

                if is_field_oriented:
                    # フィールド基準操縦:
                    cos_y = math.cos(self.current_yaw)
                    sin_y = math.sin(self.current_yaw)
                    v_x_local = v_x_field * cos_y + v_y_field * sin_y
                    v_y_local = -v_x_field * sin_y + v_y_field * cos_y
                else:
                    # ロボットローカル基準操縦 (自己位置のYawに依存せず、スティック方向へ直接進む)
                    v_x_local = v_x_field
                    v_y_local = v_y_field

                vx = int(v_x_local * VEL_SCALE)
                vy = int(v_y_local * VEL_SCALE)

                data = struct.pack('>hhh', vx, vy, vz)
                self._send_can(0x510, data)

                # 手動モード時のみボタン情報のCAN送信 (0x500, 0x501, 0x502)
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
        """CAN送信データをcan_nodeへパブリッシュ (1ID毎に1ms休止)"""
        msg = Int32MultiArray()
        msg.data = [can_id] + list(data)
        self.can_pub.publish(msg)
        with self.lock:
            self.state['last_can'] = (
                f"ID:0x{can_id:03X} Data:{list(data)}"
            )
        time.sleep(0.001)

    def _print_display(self):
        with self.lock:
            sys.stdout.write('\033[2J\033[H')
            sys.stdout.write("=" * 52 + "\n")
            sys.stdout.write(
                f" ROBOWARE NODE | Mode: {self.state['mode']} ({self.control_style})\n"
            )
            sys.stdout.write(
                f"               | Yaw:  {math.degrees(self.current_yaw):>6.1f} deg (Odom Rx: {self.odom_count})\n"
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
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
