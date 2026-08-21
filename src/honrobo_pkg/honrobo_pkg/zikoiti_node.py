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

            # CANable (SocketCAN用) は絶対に対象外
            if 'canable' in desc or '16d0:117e' in hwid:
                continue
            if used_can_port and p.device == used_can_port:
                continue

            # STM32 や STLink や Arduino らしきキーワードがあれば即決
            if any(k in desc or k in hwid for k in ['stlink', 'st-link', 'stm32', 'arduino', 'ch340', 'cp210']):
                target_port = p.device
                break

        # 2次探索: 見つからなかった場合のフォールバック（CANableと使用中ポートを除いた最初のACM/USB）
        if not target_port:
            for p in ports:
                desc = p.description.lower()
                hwid = p.hwid.lower()

                if 'canable' in desc or '16d0:117e' in hwid:
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
    """Arduino(OTOS) + ジャイロ 統合オドメトリノード"""

    def __init__(self):
        super().__init__('zikoiti_node')
        self.declare_parameter('use_microcontroller', False)
        self.use_micro = self.get_parameter('use_microcontroller').value or _use_micro

        self.port = _arduino_port
        self.ser = None
        
        # 接続管理用
        self.reconnect_cooldown = 1.0  # 再接続の試行間隔 (秒)
        self.last_reconnect_time = 0.0

        if self.port:
            try:
                self.ser = serial.Serial(self.port, 115200, timeout=0.1)
            except Exception as e:
                with _output_lock:
                    _latest_lines['arduino_err'] = f"[ARDUINO INIT] {e}"
                self.ser = None

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # 統合位置計算用
        self.prev_x_raw = None
        self.prev_y_raw = None
        self.prev_theta_arduino = None

        # ジャイロ角度を考慮した真の座標
        self.true_x = 0.0
        self.true_y = 0.0
        self.tf_broadcaster = TransformBroadcaster(self)

        # 追加データ保持用 (STM32等からの拡張データ)
        self.position_mode = 0
        self.e1_dist = 0.0
        self.e2_dist = 0.0
        self.e3_dist = 0.0
        self.e4_dist = 0.0

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
                    if "Permission denied" in err_str or "PermissionError" in err_str or "[Errno 13]" in err_str:
                        _latest_lines['arduino_err'] = (
                            f"[{tag} PERMISSION ERROR] {self.port} の読み書き権限がありません。\n"
                            "  【対策】以下のセットアップスクリプトを実行してください：\n"
                            "  bash scripts/setup_serial_rules.sh"
                        )
                    else:
                        _latest_lines['arduino_err'] = f"[{tag} CONNECT ERROR] {self.port} への接続失敗: {e}"
                self.ser = None
                return

        # 接続中の受信データ処理
        try:
            # in_waiting のチェック自体でシリアル切断時に例外が発生することがあります
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line or line.startswith('#'):
                    return

                # 画面表示用にRAWの受信行を保存
                with _output_lock:
                    _latest_lines['raw_rx'] = line

                # 1. まず、あらゆるラベル（X:, Y:, mm, degなど）にマッチして個別に数値を抽出できるか試みる
                x_match = re.search(r'x\s*:\s*(-?\d+(?:\.\d+)?)', line, re.IGNORECASE)
                y_match = re.search(r'y\s*:\s*(-?\d+(?:\.\d+)?)', line, re.IGNORECASE)
                yaw_match = re.search(r'(yaw|head|z|yaw_deg|yaw_rad|w|omega|gyro)\s*:\s*(-?\d+(?:\.\d+)?)', line, re.IGNORECASE)

                if x_match and y_match and yaw_match:
                    val1 = float(x_match.group(1))
                    val2 = float(y_match.group(1))
                    val3 = float(yaw_match.group(2))
                else:
                    # 2. 個別ラベルが無い場合、あるいはマッチしない場合
                    # カンマ区切りの文字列から数字だけを抽出してパースする
                    parts = line.split(',')
                    if len(parts) >= 3:
                        val1_nums = re.findall(r'[-+]?\d*\.\d+|\d+', parts[0])
                        val2_nums = re.findall(r'[-+]?\d*\.\d+|\d+', parts[1])
                        val3_nums = re.findall(r'[-+]?\d*\.\d+|\d+', parts[2])
                        if val1_nums and val2_nums and val3_nums:
                            val1 = float(val1_nums[0])
                            val2 = float(val2_nums[0])
                            val3 = float(val3_nums[0])
                        else:
                            return
                    else:
                        # 3. カンマが無く、スペース区切りやその他のノイズ混じり文字列の場合の最終手段
                        all_nums = re.findall(r'[-+]?\d*\.\d+|\d+', line)
                        if len(all_nums) >= 3:
                            val1 = float(all_nums[0])
                            val2 = float(all_nums[1])
                            val3 = float(all_nums[2])
                        else:
                            return

                # 拡張データの抽出 (もし存在すれば)
                mode_match = re.search(r'mode\s*:\s*(-?\d+)', line, re.IGNORECASE)
                e1_match = re.search(r'e1\s*:\s*(-?\d+(?:\.\d+)?)', line, re.IGNORECASE)
                e2_match = re.search(r'e2\s*:\s*(-?\d+(?:\.\d+)?)', line, re.IGNORECASE)
                e3_match = re.search(r'e3\s*:\s*(-?\d+(?:\.\d+)?)', line, re.IGNORECASE)
                e4_match = re.search(r'e4\s*:\s*(-?\d+(?:\.\d+)?)', line, re.IGNORECASE)

                if mode_match:
                    self.position_mode = int(mode_match.group(1))
                if e1_match:
                    self.e1_dist = float(e1_match.group(1))
                if e2_match:
                    self.e2_dist = float(e2_match.group(1))
                if e3_match:
                    self.e3_dist = float(e3_match.group(1))
                if e4_match:
                    self.e4_dist = float(e4_match.group(1))

                if self.use_micro:
                    # 💡 外部マイコン直接自己位置受信モード (ミリメートル単位・度数法単位に固定)
                    self.true_x = val1 / 1000.0
                    self.true_y = val2 / 1000.0
                    z_deg = val3
                    z_rad = math.radians(val3)

                    # 外部用のターミナル表示文言
                    combined = (
                        f"EXTERNAL MODE (Microcontroller Serial)\n"
                        f"  [RAW RX] {line}\n"
                        f"  [PARSED] X: {self.true_x:>6.3f} m, Y: {self.true_y:>6.3f} m, Yaw: {z_deg:>7.2f} °\n"
                        f"  [STATUS] Mode: {self.position_mode}\n"
                        f"  [ENCODERS] E1: {self.e1_dist:.1f} | E2: {self.e2_dist:.1f} | E3: {self.e3_dist:.1f} | E4: {self.e4_dist:.1f}"
                    )
                else:
                    # 💡 従来モード (OTOS + ジャイロ統合)
                    # ジャイロはマイコンに直接接続されており、val3 (head_deg) として一緒に送信されます
                    x_inch, y_inch, head_deg = val1, val2, val3
                    x_m_raw = x_inch * 0.0254
                    y_m_raw = y_inch * 0.0254
                    theta_arduino = math.radians(head_deg)

                    # 初回: 現在値を基準点として保存
                    if self.prev_x_raw is None:
                        self.prev_x_raw = x_m_raw
                        self.prev_y_raw = y_m_raw
                        self.prev_theta_arduino = theta_arduino

                    # Z(ω) はマイコンから送られてきたジャイロの値 (head_deg) をそのまま使用
                    z_deg = head_deg
                    z_rad = theta_arduino

                    # 1. Arduino座標系での移動量（差分）
                    delta_x_ard = x_m_raw - self.prev_x_raw
                    delta_y_ard = y_m_raw - self.prev_y_raw

                    # 2. ロボットローカル座標系への逆変換
                    cos_a = math.cos(self.prev_theta_arduino)
                    sin_a = math.sin(self.prev_theta_arduino)
                    local_dx = delta_x_ard * cos_a + delta_y_ard * sin_a
                    local_dy = -delta_x_ard * sin_a + delta_y_ard * cos_a

                    # 3. ジャイロ角度でワールド座標系へ変換
                    cos_g = math.cos(z_rad)
                    sin_g = math.sin(z_rad)
                    true_dx = local_dx * cos_g - local_dy * sin_g
                    true_dy = local_dx * sin_g + local_dy * cos_g

                    # 4. 真の座標を更新
                    self.true_x += true_dx
                    self.true_y += true_dy

                    # 次回計算用に保存
                    self.prev_x_raw = x_m_raw
                    self.prev_y_raw = y_m_raw
                    self.prev_theta_arduino = theta_arduino

                    # 内部用のターミナル表示文言
                    combined = (
                        f"INTERNAL FUSION MODE (OTOS + Gyro via Arduino)\n"
                        f"  [RAW RX] {line}\n"
                        f"  [FUSED]  X: {self.true_x:>6.3f} m, Y: {self.true_y:>6.3f} m, Gyro: {z_deg:>7.2f} °"
                    )

                with _output_lock:
                    _latest_lines['combined'] = combined

                # ROS 2 Odometry 配信
                msg = Odometry()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'odom'
                msg.child_frame_id = 'base_link'
                msg.pose.pose.position.x = self.true_x
                msg.pose.pose.position.y = self.true_y
                msg.pose.pose.orientation = self._euler_to_quat(0, 0, z_rad)
                self.odom_pub.publish(msg)

                # TF ブロードキャスト (odom -> base_link)
                t = TransformStamped()
                t.header.stamp = msg.header.stamp
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_link'
                t.transform.translation.x = self.true_x
                t.transform.translation.y = self.true_y
                t.transform.translation.z = 0.0
                t.transform.rotation = msg.pose.pose.orientation
                self.tf_broadcaster.sendTransform(t)

        except ValueError as e:
            with _output_lock:
                _latest_lines['arduino_err'] = f"[PARSE ERROR] {e}"
        except UnicodeDecodeError as e:
            with _output_lock:
                _latest_lines['arduino_err'] = f"[DECODE ERROR] {e}"
        except (serial.SerialException, OSError) as e:
            # 物理的な切断（マイコンの取り外し等）を検知してクローズ＆再スキャン移行
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
    global _should_exit, _use_micro, manual_gyro_port, manual_arduino_port

    # 引数から外部マイコン直接自己位置使用モードであるかを判別
    _use_micro = '--use-micro' in sys.argv or any('use_microcontroller:=true' in arg for arg in sys.argv)

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

                # 画面を上書き（カーソルを左上へ）
                sys.stdout.write('\033[H')
                sys.stdout.write("====================================================\n")
                if _use_micro:
                    sys.stdout.write("  ZIKOITI NODE | Mode: EXTERNAL (Microcontroller Serial)\n")
                else:
                    sys.stdout.write("  ZIKOITI NODE | Mode: INTERNAL (OTOS + Gyro via Arduino)\n")
                sys.stdout.write("====================================================\n")
                sys.stdout.write(f"[PORT STATUS] {_status_message}\n")
                sys.stdout.write("----------------------------------------------------\n")
                sys.stdout.write(f"[ESTIMATION]\n{combined}\n")
                sys.stdout.write("----------------------------------------------------\n")

                if arduino_err:
                    sys.stdout.write("[ERROR LOGS]\n")
                    sys.stdout.write(f"  Serial: {arduino_err}\n")
                else:
                    sys.stdout.write("\033[K\n\033[K\n")
                sys.stdout.write("====================================================\n")
                sys.stdout.flush()

            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        _should_exit = True


if __name__ == '__main__':
    main()
