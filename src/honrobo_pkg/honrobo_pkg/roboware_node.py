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

MAX_SPEED = 1000.0        # 最大並進速度 (mm/s) - 1.0 m/s
MAX_ANGULAR = 90.0        # 最大回転速度 (deg/s) - 90 deg/s
VEL_SCALE = 10.0          # CAN送信時のスケール倍率

# 33kgオムニ4輪 (マブチ555 24V) 向け 台形加減速パラメータ
ACCEL_XY = 1200         # 並進加速度 (mm/s^2)
DECEL_XY = 3000       # 並進減速度 (mm/s^2)
ACCEL_ROT = 75.0          # 旋回加速度 (deg/s^2)
DECEL_ROT = 112.5         # 旋回減速度 (deg/s^2)


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
        self.speed_scale = 1.0
        self.field_oriented_mode = False  # モードのトグル状態 (True: FIELD / False: LOCAL)
        self.prev_triangle_state = 0      # 三角ボタンの前回の状態

        # 33kgオムニ4輪用 台形加減速スルーレート制御状態
        self.cur_vx_local = 0.0
        self.cur_vy_local = 0.0
        self.cur_vz = 0.0
        self.last_ramp_time = time.time()
        self.last_joy_time = 0.0
        self.last_nav_time = 0.0
        self.joy_msg_count = 0
        self.can_tx_count = 0

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
        self.last_nav_time = time.time()
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
        self.joy_msg_count += 1
        self.last_joy_time = time.time()
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
            # ── 自動運転モード (/nav_cmd 入力に台形加減速を適用) ──
            is_nav_valid = (self.latest_nav_msg is not None) and ((now - self.last_nav_time) < 0.5)
            if is_nav_valid:
                msg = self.latest_nav_msg
                target_vx_local = msg.linear.x * 1000.0
                target_vy_local = msg.linear.y * 1000.0
                vz_target = math.degrees(msg.angular.z)
            else:
                target_vx_local = 0.0
                target_vy_local = 0.0
                vz_target = 0.0
            raw_btns = []
            self.control_style = "AUTO"
        else:
            # ── 手動操縦モード ──
            # コントローラーが接続され、直近0.5秒以内に受信があるか判定
            is_joy_valid = (self.latest_joy_msg is not None) and ((now - self.last_joy_time) < 0.5)

            if is_joy_valid:
                msg = self.latest_joy_msg
                
                # L1ボタン(インデックス4)が押されている場合は低速・微調整モード (例: 25%の速度)
                slow_mode = (len(msg.buttons) > 4 and msg.buttons[4] == 1)
                self.speed_scale = 0.25 if slow_mode else 1.0

                v_x_field = -msg.axes[0] * MAX_SPEED * self.speed_scale
                v_y_field = msg.axes[1] * MAX_SPEED * self.speed_scale
                vz_target = msg.axes[2] * MAX_ANGULAR * self.speed_scale

                is_field_oriented = self.field_oriented_mode
                base_style = "FIELD" if is_field_oriented else "LOCAL"
                self.control_style = f"{base_style} (SLOW)" if slow_mode else base_style

                if is_field_oriented:
                    # フィールド基準操縦 (v_y_field: +Xフィールド直進, v_x_field: +Yフィールド横移動)
                    yaw_calc = self.current_yaw
                    cos_y = math.cos(yaw_calc)
                    sin_y = math.sin(yaw_calc)
                    target_vy_local =  v_y_field * cos_y + v_x_field * sin_y
                    target_vx_local = -v_y_field * sin_y + v_x_field * cos_y
                else:
                    # ロボットローカル基準操縦
                    target_vx_local = v_x_field
                    target_vy_local = v_y_field

                raw_btns = list(msg.buttons)
            else:
                # コントローラー未接続またはタイムアウト時は目標速度0 & ボタン全0
                target_vx_local = 0.0
                target_vy_local = 0.0
                vz_target = 0.0
                raw_btns = []

        # 4輪オムニ車輪最大速度の飽和防止 (車輪合成速度がMAX_SPEEDを超えないようにスケーリング)
        # 車輪表面速度: |V_trans| + R * |omega| <= MAX_SPEED
        ROBOT_RADIUS_MM = 350.0  # 中心からホイールまでの実効距離 (mm)
        rot_lin_speed = abs(math.radians(vz_target)) * ROBOT_RADIUS_MM
        trans_speed = math.hypot(target_vx_local, target_vy_local)
        total_wheel_speed = trans_speed + rot_lin_speed
        if total_wheel_speed > MAX_SPEED and total_wheel_speed > 1e-3:
            scale = MAX_SPEED / total_wheel_speed
            target_vx_local *= scale
            target_vy_local *= scale
            vz_target *= scale

        # 33kgオムニ4輪・マブチ555向け 台形加減速スルーレート制御を適用 (手動・自動ともに適用)
        ramp_vx, ramp_vy, ramp_vz = self._apply_ramp(
            target_vx_local, target_vy_local, vz_target, dt
        )

        vx = int(round(ramp_vx * VEL_SCALE))
        vy = int(round(ramp_vy * VEL_SCALE))
        vz = int(round(ramp_vz * VEL_SCALE))
        vx = max(-32767, min(32767, vx))
        vy = max(-32767, min(32767, vy))
        vz = max(-32767, min(32767, vz))

        # ① 速度指令 (0x510) を必ず100Hzで定周期パブリッシュ
        data = struct.pack('>hhh', vx, vy, vz)
        self._send_can(0x510, data)

        # ② ボタン情報 (0x500, 0x501, 0x502) を必ず100Hzで定周期パブリッシュ
        btns = list(raw_btns)
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

    def _send_can(self, can_id, data):
        """CAN送信データをcan_nodeへパブリッシュ (高速ノンブロッキング)"""
        msg = Int32MultiArray()
        msg.data = [can_id] + list(data)
        self.can_pub.publish(msg)
        self.can_tx_count += 1
        with self.lock:
            self.state['last_can'] = (
                f"ID:0x{can_id:03X} Data:{list(data)}"
            )

    def _print_display(self):
        now = time.time()
        with self.lock:
            sys.stdout.write('\033[2J\033[H')
            sys.stdout.write("=" * 56 + "\n")
            sys.stdout.write(
                f" ROBOWARE NODE | Mode: {self.state['mode']} ({self.control_style})\n"
            )
            sys.stdout.write(
                f"               | Yaw:  {math.degrees(self.current_yaw):>6.1f} deg (Odom Rx: {self.odom_count})\n"
            )
            sys.stdout.write("=" * 56 + "\n")

            # Joy接続状態の可視化
            if self.joy_msg_count > 0 and (now - self.last_joy_time) < 0.5:
                joy_status = f"CONNECTED ({self.joy_msg_count} msgs, {(now - self.last_joy_time)*1000:.0f}ms ago)"
            elif self.joy_msg_count > 0:
                joy_status = f"TIMEOUT (last {(now - self.last_joy_time):.1f}s ago)"
            else:
                joy_status = "WAITING FOR /ps4_joy..."
            sys.stdout.write(f"[JOY STATUS] {joy_status}\n")

            sys.stdout.write("[STICKS]\n")
            for i, lbl in enumerate(AXIS_LABELS):
                val = (self.state['axes'][i]
                       if i < len(self.state['axes']) else 0.0)
                if lbl in ('L2', 'R2'):
                    sys.stdout.write(f"  {lbl}: {val:5.2f} |")
                elif lbl == 'RX':
                    spd = val * MAX_ANGULAR * self.speed_scale
                    sys.stdout.write(f"  {lbl}: {spd:>6.1f} deg/s|")
                else:
                    spd = val * MAX_SPEED * self.speed_scale
                    sys.stdout.write(f"  {lbl}: {spd:>6.1f} mm/s |")
                if i % 2 == 1:
                    sys.stdout.write("\n")

            btns = (", ".join(self.state['buttons'])
                    if self.state['buttons'] else "None")
            sys.stdout.write(f"[BUTTONS]  {btns}\n")
            sys.stdout.write(f"[NAV CMD]  {self.state['nav_cmd']}\n")
            sys.stdout.write(
                f"[RAMP OUT] VX:{self.cur_vx_local:>6.1f} VY:{self.cur_vy_local:>6.1f} "
                f"VZ:{self.cur_vz:>5.1f} deg/s (Max: {MAX_SPEED * self.speed_scale:.0f} mm/s)\n"
            )
            sys.stdout.write(f"[CAN TX]   {self.state['last_can']} (Total: {self.can_tx_count})\n")
            sys.stdout.write("=" * 56 + "\n")
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
