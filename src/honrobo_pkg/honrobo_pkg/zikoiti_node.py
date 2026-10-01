"""
zikoiti_node.py — 自己位置推定ノード

IMU(WT901ジャイロ) と Arduino(OTOSセンサー) のデータを統合し、
高精度な自己位置推定を行います。

- ジャイロ: Z軸角度（ヨー角）を高精度に提供
- Arduino(OTOS): X, Y の移動量を提供
- 統合: ArduinoのX,Y移動量をローカル座標に逆変換後、ジャイロ角度でワールド座標に変換

配信トピック:
    /odom (nav_msgs/Odometry) — 位置 (x, y) + 姿勢 (quaternion)

シリアルポートの自動検出・権限付与を自動で行います。
"""

import re
import sys
import os
import threading
import signal
import time
import serial
import serial.tools.list_ports
import struct
import math
import subprocess
from collections import defaultdict

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


# ============================================================
# グローバル状態
# ============================================================
_output_lock = threading.Lock()
_latest_lines = defaultdict(str)
_should_exit = False

# 外部マイコン直接自己位置使用フラグ
_use_micro = False

# ジャイロ/Yaw回転方向反転フラグ (デフォルト: False)
_invert_yaw = False

# 自動検出されたポート
_arduino_port = None
_status_message = "ポート検索中..."


# ============================================================
# ユーティリティ関数
# ============================================================
def normalize_angle(angle):
    """角度を -180 ~ 180 度に正規化する"""
    while angle > 180:
        angle -= 360
    while angle <= -180:
        angle += 360
    return angle


manual_arduino_port = None


def auto_detect_ports():
    """自動でArduino/外部マイコンのポートを探す（手動指定優先）"""
    global _arduino_port, _status_message, manual_arduino_port
    
    # 1. 手動指定されている場合はそちらを最優先
    _arduino_port = manual_arduino_port

    if _arduino_port:
        # 必要なポートが手動指定されていれば自動検出は不要
        pass
    else:
        ports = serial.tools.list_ports.comports()

        # 使用中のCANポートを取得
        used_can_port = None
        try:
            if os.path.exists('/tmp/honrobo_can_port'):
                with open('/tmp/honrobo_can_port', 'r') as f:
                    used_can_port = f.read().strip()
        except Exception:
            pass

        # 1次探索: STM32(STLink VCP) または Arduino らしいポートを優先検出
        target_port = None
        for p in ports:
            desc = p.description.lower()
            hwid = p.hwid.lower()

            # CANable (SocketCAN用) と RPLIDAR は絶対に対象外
            if 'canable' in desc or '16d0:117e' in hwid:
                continue
            if '10c4:ea60' in hwid or 'rplidar' in desc:
                continue
            if used_can_port and p.device == used_can_port:
                continue

            # STM32 や STLink や Arduino らしきキーワードがあれば即決
            if any(k in desc or k in hwid for k in ['stlink', 'st-link', 'stm32', 'arduino', 'ch340', 'cp210']):
                target_port = p.device
                break

        # 2次探索: 見つからなかった場合のフォールバック
        if not target_port:
            for p in ports:
                desc = p.description.lower()
                hwid = p.hwid.lower()

                if 'canable' in desc or '16d0:117e' in hwid:
                    continue
                if '10c4:ea60' in hwid or 'rplidar' in desc:
                    continue
                if used_can_port and p.device == used_can_port:
                    continue

                if 'acm' in p.device.lower() or 'usb' in p.device.lower():
                    target_port = p.device
                    break

        _arduino_port = target_port

    status = []
    if _use_micro:
        status.append(f"Microcontroller (STM32): {_arduino_port or '未検出'}")
    else:
        status.append(f"Arduino (OTOS+Gyro): {_arduino_port or '未検出'}")
    _status_message = " | ".join(status)


def setup_permissions():
    """シリアルデバイス of アクセス権限を確認し、無ければ警告を出す"""
    devices = []
    if _arduino_port:
        devices.append(_arduino_port)

    for dev in devices:
        if os.path.exists(dev):
            if not os.access(dev, os.R_OK | os.W_OK):
                with _output_lock:
                    _latest_lines['arduino_err'] = (
                        f"[PERMISSION ERROR] {dev} の読み書き権限がありません。"
                        "sudo usermod -aG dialout $USER を実行し再ログインしてください。"
                    )





# ============================================================
# ROS 2 自己位置推定ノード
# ============================================================
class OtosOdomNode(Node):
    """Arduino(OTOS) + ジャイロ 統合オドメトリノード
    
    holo_mcu_bridge スタイルの絶対位置追跡方式を採用。
    差分計算による累積誤差・90度ズレバグを根本解決。
    """

    def __init__(self):
        super().__init__('zikoiti_node')
        self.declare_parameter('use_microcontroller', False)
        self.use_micro = self.get_parameter('use_microcontroller').value or _use_micro
        self.declare_parameter('invert_yaw', _invert_yaw)
        self.invert_yaw = bool(self.get_parameter('invert_yaw').value)
        self.declare_parameter('yaw_offset', 90.0)
        self.yaw_offset = float(self.get_parameter('yaw_offset').value)

        self.port = _arduino_port
        self.ser = None
        
        # 接続管理用
        self.reconnect_cooldown = 1.0
        self.last_reconnect_time = 0.0

        if self.port:
            try:
                self.ser = serial.Serial(self.port, 115200, timeout=0.1)
            except Exception as e:
                with _output_lock:
                    _latest_lines['arduino_err'] = f"[ARDUINO INIT] {e}"
                self.ser = None

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── holo_mcu_bridge スタイルの絶対位置追跡 ──
        # マイコンが送ってくる「生の絶対座標」と「出力する座標」の差（オフセット）
        self.offset_x_m = 0.0
        self.offset_y_m = 0.0
        self.offset_yaw_rad = 0.0
        self.has_received_data = False
        self.last_raw_x_m = 0.0
        self.last_raw_y_m = 0.0
        self.last_raw_yaw_rad = 0.0

        # 速度計算用
        self.last_pub_time = None
        self.last_pub_x_m = 0.0
        self.last_pub_y_m = 0.0
        self.last_pub_yaw_rad = 0.0

        # 追加データ保持用
        self.position_mode = 0
        self.e1_dist = 0.0
        self.e2_dist = 0.0
        self.e3_dist = 0.0
        self.e4_dist = 0.0

        # パーサー用 regex (holo_mcu_bridge と同一)
        import re as _re
        self._pat_full = _re.compile(
            r"X:([-\d\.]+)\s+Y:([-\d\.]+)\s+Yaw:([-\d\.]+)"
        )

    def _normalize_angle_rad(self, a):
        while a > math.pi:  a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def update(self):
        """シリアルからデータを読み取り、オドメトリを計算・配信する"""
        current_time = time.time()

        # ポート接続がない、もしくは切断された場合、再検出と接続を試みる
        if not self.ser:
            if current_time - self.last_reconnect_time < self.reconnect_cooldown:
                return
            self.last_reconnect_time = current_time

            # ポートの再スキャン
            auto_detect_ports()
            self.port = _arduino_port

            if not self.port:
                with _output_lock:
                    tag = "MICRO" if self.use_micro else "ARDUINO"
                    _latest_lines['arduino_err'] = f"[{tag} SEARCHING] 接続可能なポートが見つかりません。捜索中..."
                return

            try:
                self.ser = serial.Serial(self.port, 115200, timeout=0.1)
                with _output_lock:
                    _latest_lines['arduino_err'] = ""
                self.get_logger().info(f"Successfully reconnected to serial port: {self.port}")
            except Exception as e:
                with _output_lock:
                    tag = "MICRO" if self.use_micro else "ARDUINO"
                    err_str = str(e)
                    if "Permission denied" in err_str or "[Errno 13]" in err_str:
                        _latest_lines['arduino_err'] = (
                            f"[{tag} PERMISSION ERROR] {self.port} の読み書き権限がありません。\n"
                            "  bash scripts/setup_serial_rules.sh を実行してください"
                        )
                    else:
                        _latest_lines['arduino_err'] = f"[{tag} CONNECT ERROR] {self.port}: {e}"
                self.ser = None
                return

        # 接続中の受信データ処理
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line or line.startswith('#'):
                    return

                with _output_lock:
                    _latest_lines['raw_rx'] = line

                self._parse_and_publish(line)

        except ValueError as e:
            with _output_lock:
                _latest_lines['arduino_err'] = f"[PARSE ERROR] {e}"
        except UnicodeDecodeError as e:
            with _output_lock:
                _latest_lines['arduino_err'] = f"[DECODE ERROR] {e}"
        except (serial.SerialException, OSError) as e:
            with _output_lock:
                tag = "MICRO" if self.use_micro else "ARDUINO"
                _latest_lines['arduino_err'] = f"[{tag} DISCONNECTED] 接続が失われました: {e}"
            self.get_logger().warn(f"Sensor microcontroller disconnected. Searching for port...")
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.port = None
        except Exception as e:
            with _output_lock:
                _latest_lines['arduino_err'] = f"[UPDATE ERROR] {e}"
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.port = None

    def _parse_and_publish(self, line: str):
        """シリアル行をパースしてオドメトリを配信 (holo_mcu_bridge スタイル)"""
        # X:xxx Y:xxx Yaw:xxx 形式を優先パース
        m = self._pat_full.search(line)
        if m:
            raw_val1 = float(m.group(1))
            raw_val2 = float(m.group(2))
            raw_yaw_deg = float(m.group(3))
        else:
            # フォールバック: カンマ区切り / スペース区切り数値
            import re as _re
            parts = line.split(',')
            if len(parts) >= 3:
                nums = [_re.findall(r'[-+]?\d*\.?\d+', p) for p in parts[:3]]
                if all(nums):
                    raw_val1, raw_val2 = float(nums[0][0]), float(nums[1][0])
                    raw_yaw_deg = float(nums[2][0])
                else:
                    return
            else:
                all_nums = _re.findall(r'[-+]?\d*\.?\d+', line)
                if len(all_nums) >= 3:
                    raw_val1, raw_val2, raw_yaw_deg = float(all_nums[0]), float(all_nums[1]), float(all_nums[2])
                else:
                    return

        # 拡張データ抽出
        import re as _re
        for attr, pattern in [('position_mode', r'[Mm]ode\s*:?\s*(-?\d+)'),
                               ('e1_dist', r'[Ee]1\s*:?\s*(-?[\d.]+)'),
                               ('e2_dist', r'[Ee]2\s*:?\s*(-?[\d.]+)'),
                               ('e3_dist', r'[Ee]3\s*:?\s*(-?[\d.]+)'),
                               ('e4_dist', r'[Ee]4\s*:?\s*(-?[\d.]+)')]:
            mm = _re.search(pattern, line)
            if mm:
                setattr(self, attr, float(mm.group(1)))

        # ── 座標変換 (holo_mcu_bridge 方式) ──
        if self.use_micro:
            # 外部マイコン直接自己位置モード: 単位 mm → m
            raw_x_m = raw_val1 / 1000.0
            raw_y_m = raw_val2 / 1000.0
        else:
            # OTOS内蔵モード: 単位 inch → m
            raw_x_m = raw_val1 * 0.0254
            raw_y_m = raw_val2 * 0.0254

        # Yaw: invert → rad
        if self.invert_yaw:
            raw_yaw_deg = -raw_yaw_deg
        raw_yaw_rad = self._normalize_angle_rad(math.radians(raw_yaw_deg))

        # 初回パケット: 起動時の生座標・生Yawを基準として記録
        # X/Yと同様に Yaw も「起動時からの変化量」に変換することで、
        # ジャイロが電源ON時にランダムな値を持っていても常に yaw_offset(90°)でスタートできる
        if not self.has_received_data:
            self.offset_x_m    = raw_x_m
            self.offset_y_m    = raw_y_m
            self.offset_yaw_rad = raw_yaw_rad   # 起動時の生Yaw角を基準に保存
            self.has_received_data = True
            self.get_logger().info(
                f"初回受信: X={raw_x_m:.3f}m, Y={raw_y_m:.3f}m, Yaw_raw={math.degrees(raw_yaw_rad):.1f}° → 基準点記録完了"
            )

        self.last_raw_x_m    = raw_x_m
        self.last_raw_y_m    = raw_y_m
        self.last_raw_yaw_rad = raw_yaw_rad

        # 1. 起動時を原点としたセンサローカルな移動量
        local_x = raw_x_m - self.offset_x_m
        local_y = raw_y_m - self.offset_y_m

        # 2. ROS座標系への回転変換
        # マイコン(OTOS)の座標系は「起動時の向き」が+X軸。
        # 一方、ROS側の初期姿勢は yaw_offset (例: 90度 = +Y軸方向)。
        # したがって、OTOSの座標をそのまま使うと進行方向が90度ズレてしまう。
        # OTOSの絶対座標を yaw_offset 分だけ回転させて ROS座標系に合わせる。
        theta0 = math.radians(self.yaw_offset)
        x_ros = local_x * math.cos(theta0) - local_y * math.sin(theta0)
        y_ros = local_x * math.sin(theta0) + local_y * math.cos(theta0)

        # Yaw = (現在の生Yaw - 起動時の生Yaw) + yaw_offset
        delta_yaw = self._normalize_angle_rad(raw_yaw_rad - self.offset_yaw_rad)
        yaw_rad   = self._normalize_angle_rad(delta_yaw + theta0)

        # 速度計算
        now_time = time.time()
        vx = vy = wz = 0.0
        if self.last_pub_time is not None:
            dt = now_time - self.last_pub_time
            if 0.001 < dt < 0.5:
                dx_g = x_ros - self.last_pub_x_m
                dy_g = y_ros - self.last_pub_y_m
                dyaw = self._normalize_angle_rad(yaw_rad - self.last_pub_yaw_rad)
                cos_y = math.cos(yaw_rad)
                sin_y = math.sin(yaw_rad)
                vx = dx_g * cos_y + dy_g * sin_y
                vy = -dx_g * sin_y + dy_g * cos_y
                wz = dyaw / dt

        self.last_pub_time = now_time
        self.last_pub_x_m = x_ros
        self.last_pub_y_m = y_ros
        self.last_pub_yaw_rad = yaw_rad

        # ターミナル表示
        inv_str = " (InvertYaw)" if self.invert_yaw else ""
        mode_str = "EXTERNAL (Microcontroller)" if self.use_micro else "INTERNAL (OTOS+Gyro)"
        combined = (
            f"MODE: {mode_str}\n"
            f"  [RAW RX] {line}\n"
            f"  [POSE]   X: {x_ros:>7.3f} m  Y: {y_ros:>7.3f} m  Yaw: {math.degrees(yaw_rad):>7.2f}°{inv_str}\n"
            f"  [STATUS] Mode: {int(self.position_mode)}  "
            f"E1:{self.e1_dist:.0f} E2:{self.e2_dist:.0f} E3:{self.e3_dist:.0f} E4:{self.e4_dist:.0f}"
        )
        with _output_lock:
            _latest_lines['combined'] = combined

        # ── ROS 2 Odometry 配信 ──
        now_ros = self.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp = now_ros
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = x_ros
        msg.pose.pose.position.y = y_ros
        msg.pose.pose.orientation = self._euler_to_quat(0, 0, yaw_rad)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        # 共分散 (holo_mcu_bridge スタイル)
        cov = [0.0] * 36
        cov[0]  = 0.001  # X
        cov[7]  = 0.001  # Y
        cov[14] = 99999.0
        cov[21] = 99999.0
        cov[28] = 99999.0
        cov[35] = 0.001  # Yaw
        msg.pose.covariance = cov
        msg.twist.covariance = cov
        self.odom_pub.publish(msg)

        # ── TF: odom → base_link ──
        t = TransformStamped()
        t.header.stamp = now_ros
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x_ros
        t.transform.translation.y = y_ros
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
    @staticmethod
    def _euler_to_quat(roll, pitch, yaw):
        """オイラー角 → クォータニオン変換"""
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        return Quaternion(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * cp * cy,  # 修正: 正しい符号
            w=cr * cp * cy + sr * sp * sy,
        )


# ============================================================
# Arduinoスレッド
# ============================================================
def arduino_thread():
    """ROS 2ノードをスレッドで実行"""
    global _should_exit

    try:
        rclpy.init()
        node = OtosOdomNode()

        while not _should_exit and rclpy.ok():
            try:
                node.update()
                rclpy.spin_once(node, timeout_sec=0.01)
                time.sleep(0.002)  # ビジーウェイト防止用のスリープ
            except Exception as e:
                with _output_lock:
                    _latest_lines['arduino_err'] = f"[LOOP ERROR] {e}"
                time.sleep(0.1)

        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    except Exception as e:
        with _output_lock:
            _latest_lines['arduino_err'] = f"[ARDUINO ERROR] {e}"


# ============================================================
# メインエントリーポイント
# ============================================================
def main():
    global _should_exit, _use_micro, _invert_yaw, manual_gyro_port, manual_arduino_port

    # 引数から外部マイコン直接自己位置使用モードであるかを判別
    _use_micro = '--use-micro' in sys.argv or any('use_microcontroller:=true' in arg.lower() for arg in sys.argv)

    # ヨー角反転設定の解析 (デフォルト: True, 右旋回で正のセンサー値をROS REP-103規格に合わせて反転)
    if '--no-invert-yaw' in sys.argv or any('invert_yaw:=false' in arg.lower() for arg in sys.argv):
        _invert_yaw = False
    elif '--invert-yaw' in sys.argv or any('invert_yaw:=true' in arg.lower() for arg in sys.argv):
        _invert_yaw = True

    # 手動指定ポートの簡易解析
    for i, arg in enumerate(sys.argv):
        if arg == '--gyro-port' and i + 1 < len(sys.argv):
            manual_gyro_port = sys.argv[i + 1]
        elif arg == '--arduino-port' and i + 1 < len(sys.argv):
            manual_arduino_port = sys.argv[i + 1]

    sys.stdout.write('\033[2J\033[H')
    sys.stdout.flush()

    print("ポートを検索中...")
    auto_detect_ports()
    setup_permissions()

    # Arduino / マイコン スレッド起動
    arduino_t = threading.Thread(target=arduino_thread, daemon=True)
    arduino_t.start()
    time.sleep(0.2)

    try:
        sys.stdout.write('\033[2J')
        while not _should_exit:
            with _output_lock:
                combined = _latest_lines.get(
                    'combined', 'Waiting for sensor data...'
                )
                arduino_err = _latest_lines.get('arduino_err', '')

                # 画面を上書き（各行末尾に \033[K を付与して残像を完全にクリア）
                buf = ["\033[H"]
                buf.append("====================================================\033[K")
                if _use_micro:
                    buf.append("  ZIKOITI NODE | Mode: EXTERNAL (Microcontroller Serial)\033[K")
                else:
                    buf.append("  ZIKOITI NODE | Mode: INTERNAL (OTOS + Gyro via Arduino)\033[K")
                buf.append("====================================================\033[K")
                buf.append(f"[PORT STATUS] {_status_message}\033[K")
                buf.append("----------------------------------------------------\033[K")
                buf.append("[ESTIMATION]\033[K")
                for c_line in combined.split('\n'):
                    buf.append(f"{c_line}\033[K")
                buf.append("----------------------------------------------------\033[K")

                if arduino_err:
                    buf.append("[ERROR LOGS]\033[K")
                    buf.append(f"  Serial: {arduino_err}\033[K")
                else:
                    buf.append("\033[K")
                    buf.append("\033[K")
                buf.append("====================================================\033[K")
                
                sys.stdout.write("\n".join(buf) + "\n")
                sys.stdout.flush()

            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        _should_exit = True


if __name__ == '__main__':
    main()
