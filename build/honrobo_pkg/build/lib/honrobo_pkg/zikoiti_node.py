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
from geometry_msgs.msg import Quaternion


# ============================================================
# グローバル状態
# ============================================================
_output_lock = threading.Lock()
_latest_lines = defaultdict(str)
_should_exit = False

# ジャイロから取得した最新の角度(Z)
_current_gyro_yaw = 0.0
_gyro_yaw_offset = None
_gyro_updated = False

# 自動検出されたポート
_gyro_port = None
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


def auto_detect_ports():
    """自動でジャイロ(ttyUSB)とArduino(ttyACM)のポートを探す"""
    global _gyro_port, _arduino_port, _status_message
    ports = serial.tools.list_ports.comports()

    _gyro_port = None
    _arduino_port = None

    for p in ports:
        # WT901などのジャイロは通常 ttyUSB として認識される
        if 'USB' in p.device:
            if _gyro_port is None:
                _gyro_port = p.device
        # Arduino (OTOS) は通常 ttyACM として認識される
        elif 'ACM' in p.device:
            if _arduino_port is None:
                _arduino_port = p.device

    status = []
    status.append(f"Gyro: {_gyro_port or '未検出(USB)'}")
    status.append(f"Arduino: {_arduino_port or '未検出(ACM)'}")
    _status_message = " | ".join(status)


def setup_permissions():
    """シリアルデバイスのアクセス権限を自動付与する"""
    devices = []
    if _gyro_port:
        devices.append(_gyro_port)
    if _arduino_port:
        devices.append(_arduino_port)

    for dev in devices:
        if os.path.exists(dev):
            if not os.access(dev, os.R_OK | os.W_OK):
                try:
                    subprocess.run(['sudo', 'chmod', '666', dev], check=True)
                except Exception:
                    pass


def transform_data(data):
    """WT901 IMU データ変換 (リトルエンディアン)"""
    return struct.unpack('<hhh', data)


# ============================================================
# ジャイロスレッド (WT901)
# ============================================================
def gyairo_thread():
    """WT901ジャイロからZ軸角度を取得し続けるスレッド"""
    global _should_exit, _current_gyro_yaw, _gyro_yaw_offset, _gyro_updated

    baud = 115200

    while not _should_exit:
        if not _gyro_port:
            time.sleep(1.0)
            continue

        try:
            ser = serial.Serial(_gyro_port, baud, timeout=0.01)
            buffer = bytearray()

            while not _should_exit:
                try:
                    waiting = ser.in_waiting
                    if waiting > 0:
                        buffer.extend(ser.read(waiting))

                        # 1パケットは11バイト
                        while len(buffer) >= 11:
                            if buffer[0] == 0x55:
                                flag = buffer[1]

                                if flag == 0x53:  # 角度データ
                                    raw_data = buffer[2:8]
                                    ax, ay, az = transform_data(bytes(raw_data))
                                    yaw = az / 32768 * 180

                                    if _gyro_yaw_offset is None:
                                        _gyro_yaw_offset = yaw

                                    _current_gyro_yaw = normalize_angle(
                                        yaw - _gyro_yaw_offset
                                    )
                                    _gyro_updated = True

                                buffer = buffer[11:]
                            else:
                                buffer.pop(0)
                    else:
                        time.sleep(0.001)
                except Exception:
                    pass
        except Exception as e:
            with _output_lock:
                _latest_lines['gyro_err'] = f"[GYRO ERROR] {e}"
            time.sleep(1.0)
        finally:
            try:
                ser.close()
            except Exception:
                pass


# ============================================================
# ROS 2 自己位置推定ノード
# ============================================================
class OtosOdomNode(Node):
    """Arduino(OTOS) + ジャイロ 統合オドメトリノード"""

    def __init__(self):
        super().__init__('zikoiti_node')
        self.port = _arduino_port
        self.ser = None

        if self.port:
            try:
                self.ser = serial.Serial(self.port, 115200, timeout=0.1)
            except Exception as e:
                with _output_lock:
                    _latest_lines['arduino_err'] = f"[ARDUINO INIT] {e}"

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # 統合位置計算用
        self.prev_x_raw = None
        self.prev_y_raw = None
        self.prev_theta_arduino = None

        # ジャイロ角度を考慮した真の座標
        self.true_x = 0.0
        self.true_y = 0.0

    def update(self):
        """Arduinoからデータを読み取り、オドメトリを計算・配信する"""
        if not self.port:
            return

        if not self.ser:
            try:
                self.ser = serial.Serial(self.port, 115200, timeout=0.1)
                with _output_lock:
                    _latest_lines['arduino_err'] = ""
            except Exception as e:
                with _output_lock:
                    _latest_lines['arduino_err'] = f"[ARDUINO RECONNECT] {e}"
                time.sleep(0.5)
                return

        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if not line or line.startswith('#'):
                    return

                x_inch, y_inch, head_deg = map(float, line.split(','))
                x_m_raw = x_inch * 0.0254
                y_m_raw = y_inch * 0.0254
                theta_arduino = math.radians(head_deg)

                # 初回: 現在値を基準点として保存
                if self.prev_x_raw is None:
                    self.prev_x_raw = x_m_raw
                    self.prev_y_raw = y_m_raw
                    self.prev_theta_arduino = theta_arduino

                # Z(ω) はジャイロの値をそのまま使用
                z_deg = _current_gyro_yaw
                z_rad = math.radians(z_deg)

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

                # ターミナル表示
                combined = (
                    f"X: {self.true_x:>6.3f} m, "
                    f"Y: {self.true_y:>6.3f} m, "
                    f"Z: {z_deg:>7.2f} °"
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

            except ValueError as e:
                with _output_lock:
                    _latest_lines['arduino_err'] = f"[PARSE ERROR] {e}"
            except UnicodeDecodeError as e:
                with _output_lock:
                    _latest_lines['arduino_err'] = f"[DECODE ERROR] {e}"
            except Exception as e:
                with _output_lock:
                    _latest_lines['arduino_err'] = f"[UPDATE ERROR] {e}"
                self.ser = None

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
    global _should_exit

    sys.stdout.write('\033[2J\033[H')
    sys.stdout.flush()

    print("ポートを検索中...")
    auto_detect_ports()
    setup_permissions()

    # ジャイロスレッド起動
    gyairo_t = threading.Thread(target=gyairo_thread, daemon=True)
    gyairo_t.start()
    time.sleep(0.2)

    # Arduinoスレッド起動
    arduino_t = threading.Thread(target=arduino_thread, daemon=True)
    arduino_t.start()
    time.sleep(0.2)

    def _sigterm_handler(signum, frame):
        global _should_exit
        _should_exit = True
        sys.exit(0)

    def _shutdown_prompt(signum=None, frame=None):
        global _should_exit
        signal.signal(signal.SIGINT, signal.SIG_IGN) # 多重割り込み防止
        
        # 描画と被らないように画面下部に空行を入れてから表示
        sys.stdout.write('\033[4;0H\033[2K')
        sys.stdout.write(f"\n\033[1;33m[Ctrl+C を検知しました: zikoiti_node]\033[0m\n")
        sys.stdout.flush()
        
        while True:
            try:
                ans = input("すべてのプログラムを終了しますか？ (a: すべて終了 / y: このプログラムのみ終了 / c: キャンセルして再開): ").strip().lower()
            except (KeyboardInterrupt, EOFError):
                ans = 'y'
            
            if ans == 'a':
                _should_exit = True
                print("すべてのプログラムを終了しています...")
                import subprocess
                subprocess.run(["bash", "/home/haru/Documents/honrobo_2026/scripts/stop_all.sh"])
                sys.exit(0)
            elif ans == 'y':
                _should_exit = True
                print("zikoiti_node を終了します...")
                sys.exit(0)
            elif ans == 'c':
                print("実行を再開します...")
                signal.signal(signal.SIGINT, _shutdown_prompt)
                # 画面を一度クリアしてメインループに戻る
                sys.stdout.write('\033[2J')
                sys.stdout.flush()
                return
            else:
                print("無効な入力です。'a', 'y', 'c' のいずれかを入力してください。")

    signal.signal(signal.SIGINT, _shutdown_prompt)
    signal.signal(signal.SIGTERM, _sigterm_handler)

    try:
        sys.stdout.write('\033[2J')
        while not _should_exit:
            with _output_lock:
                combined = _latest_lines.get(
                    'combined', 'Waiting for sensor data...'
                )
                gyro_err = _latest_lines.get('gyro_err', '')
                arduino_err = _latest_lines.get('arduino_err', '')

                sys.stdout.write('\033[1;0H\033[2K')
                sys.stdout.write(f"[PORT STATUS] {_status_message}\n")

                sys.stdout.write('\033[2;0H\033[2K')
                sys.stdout.write(f"[FUSED POSITION] {combined}\n")

                if gyro_err or arduino_err:
                    sys.stdout.write('\033[3;0H\033[2K')
                    sys.stdout.write(f"{gyro_err} {arduino_err}\n")

                sys.stdout.flush()

            time.sleep(0.05)
    except KeyboardInterrupt:
        _shutdown_prompt()


if __name__ == '__main__':
    main()
