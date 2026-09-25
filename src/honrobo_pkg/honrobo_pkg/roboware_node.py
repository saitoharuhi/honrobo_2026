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

MAX_SPEED = 1000.0       # 最大並進速度 (mm/s) - 1.0 m/s
MAX_ANGULAR = 180.0     # 最大回転速度 (deg/s) - 2倍に高速化 (1秒で半回転/180度)
VEL_SCALE = 10.0        # CAN送信時のスケール倍率

# 33kgオムニ4輪 (マブチ555 24V) 向け 台形加減速パラメータ
ACCEL_XY = 1500.0       # 並進加速度 (mm/s^2) -> 0から1000mm/sまで約0.67秒
DECEL_XY = 2200.0       # 並進減速度 (mm/s^2) -> 1000mm/sから停止まで約0.45秒
ACCEL_ROT = 300.0       # 旋回加速度 (deg/s^2) -> 約0.60秒で最高旋回(180deg/s)へ
DECEL_ROT = 450.0       # 旋回減速度 (deg/s^2) -> 約0.40秒で素早くスリップレス停止


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

        # 33kgオムニ4輪用 台形加減速スルーレート制御状態
        self.cur_vx_local = 0.0
        self.cur_vy_local = 0.0
        self.cur_vz = 0.0
        self.last_ramp_time = time.time()

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
                self.cur_vx_local = 0.0
                self.cur_vy_local = 0.0
                self.cur_vz = 0.0
                data = struct.pack('>hhh', 0, 0, 0)
                self._send_can(0x510, data)

                with self.lock:
                    self.state['mode'] = 'MANUAL (EMERGENCY STOP)'

    def _apply_ramp(self, target_vx, target_vy, target_vz, dt):
        """33kgオムニ4輪 (マブチ555 24V) の慣性とスリップを防ぐ2Dベクトル台形加減速制御"""
        if dt <= 0.0:
            return self.cur_vx_local, self.cur_vy_local, self.cur_vz
        if dt > 0.1:
            dt = 0.01

        # ── 1. 並進 (XY) 2Dベクトル台形加減速 ──
        dx = target_vx - self.cur_vx_local
        dy = target_vy - self.cur_vy_local
        dist = math.hypot(dx, dy)

        if dist > 1e-3:
            cur_speed = math.hypot(self.cur_vx_local, self.cur_vy_local)
            target_speed = math.hypot(target_vx, target_vy)
            dot = self.cur_vx_local * target_vx + self.cur_vy_local * target_vy

            # 減速または反転判定
            is_decel = (target_speed < cur_speed) or (cur_speed > 10.0 and dot < 0)
            max_accel = DECEL_XY if is_decel else ACCEL_XY
            max_step = max_accel * dt

            if dist > max_step:
                self.cur_vx_local += (dx / dist) * max_step
                self.cur_vy_local += (dy / dist) * max_step
            else:
                self.cur_vx_local = target_vx
                self.cur_vy_local = target_vy
        else:
            self.cur_vx_local = target_vx
            self.cur_vy_local = target_vy

        # 停止付近の微小ハンチング防止
        if target_vx == 0.0 and target_vy == 0.0 and math.hypot(self.cur_vx_local, self.cur_vy_local) < 5.0:
            self.cur_vx_local = 0.0
            self.cur_vy_local = 0.0

        # ── 2. 旋回 (Z) 台形加減速 ──
        dz = target_vz - self.cur_vz
        if abs(dz) > 1e-3:
            cur_rot = abs(self.cur_vz)
            target_rot = abs(target_vz)
            is_rot_decel = (target_rot < cur_rot) or (self.cur_vz * target_vz < 0)
            max_rot_accel = DECEL_ROT if is_rot_decel else ACCEL_ROT
            max_rot_step = max_rot_accel * dt

            if abs(dz) > max_rot_step:
                self.cur_vz += math.copysign(max_rot_step, dz)
            else:
                self.cur_vz = target_vz
        else:
            self.cur_vz = target_vz

        if target_vz == 0.0 and abs(self.cur_vz) < 0.5:
            self.cur_vz = 0.0

        return self.cur_vx_local, self.cur_vy_local, self.cur_vz

    def _can_tx_timer(self):
        """100Hz (0.01秒周期 / 10ms) でCANデータを定周期パブリッシュ"""
        now = time.time()
        dt = now - self.last_ramp_time
        self.last_ramp_time = now

        if self.auto_mode:
            # 自動運転中は手動の台形制御状態をリセット
            self.cur_vx_local = 0.0
            self.cur_vy_local = 0.0
            self.cur_vz = 0.0

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
                # 手動モード時のスティック→目標速度 (MAX_SPEED = 1000 mm/s)
                v_x_field = -msg.axes[0] * MAX_SPEED
                v_y_field = msg.axes[1] * MAX_SPEED
                vz_target = msg.axes[2] * MAX_ANGULAR

                # 三角ボタンで切り替えたモード状態を使用する
                is_field_oriented = self.field_oriented_mode
                self.control_style = "FIELD" if is_field_oriented else "LOCAL"

                if is_field_oriented:
                    # フィールド基準操縦:
                    yaw_calc = self.current_yaw - math.pi / 2.0
                    cos_y = math.cos(yaw_calc)
                    sin_y = math.sin(yaw_calc)
                    target_vx_local = v_x_field * cos_y + v_y_field * sin_y
                    target_vy_local = -v_x_field * sin_y + v_y_field * cos_y
                else:
                    # ロボットローカル基準操縦 (自己位置のYawに依存せず、スティック方向へ直接進む)
                    target_vx_local = v_x_field
                    target_vy_local = v_y_field

                # 33kgオムニ4輪・マブチ555向け 台形加減速スルーレート制御を適用
                ramp_vx, ramp_vy, ramp_vz = self._apply_ramp(
                    target_vx_local, target_vy_local, vz_target, dt
                )

                vx = int(ramp_vx * VEL_SCALE)
                vy = int(ramp_vy * VEL_SCALE)
                vz = int(ramp_vz * VEL_SCALE)

                data = struct.pack('>hhh', vx, vy, vz)
                self._send_can(0x510, data)

                # 手動モード時のみボタン情報のCAN送信 (0x500, 0x501, 0x502)
                if len(msg.buttons) > 0:
                    btns = list(msg.buttons)
                    if len(btns) < 17:
                        btns += [0] * (17 - len(btns))

                    # 0x500: ○△×□ + 矢印
                    b500 = [
                        btns[2], btns[3],
                        btns[1], btns[0],
                        btns[13], btns[14],
                        btns[15], btns[16],
                    ]
                    self._send_can(0x500, b500)

                    # 0x501: R1,R2,R3,L1,L2,L3
                    b501 = [
                        btns[5], btns[7], btns[12],
                        btns[4], btns[6], btns[11],
                        0, 0,
                    ]
                    self._send_can(0x501, b501)

                    # 0x502: Share, Options, PS
                    b502 = [
                        btns[8], btns[9], btns[10],
                        0, 0, 0, 0, 0,
                    ]
                    self._send_can(0x502, b502)

            elif self.cur_vx_local != 0.0 or self.cur_vy_local != 0.0 or self.cur_vz != 0.0:
                # コントローラー未受信時の滑らかな減速停止
                ramp_vx, ramp_vy, ramp_vz = self._apply_ramp(0.0, 0.0, 0.0, dt)
                vx = int(ramp_vx * VEL_SCALE)
                vy = int(ramp_vy * VEL_SCALE)
                vz = int(ramp_vz * VEL_SCALE)
                data = struct.pack('>hhh', vx, vy, vz)
                self._send_can(0x510, data)

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
            sys.stdout.write(f"[NAV CMD]  {self.state['nav_cmd']}\n")
            sys.stdout.write(
                f"[RAMP OUT] VX:{self.cur_vx_local:>6.1f} VY:{self.cur_vy_local:>6.1f} "
                f"VZ:{self.cur_vz:>5.1f} deg/s (Max: {MAX_SPEED:.0f} mm/s)\n"
            )
            sys.stdout.write(f"[CAN TX]   {self.state['last_can']}\n")
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
