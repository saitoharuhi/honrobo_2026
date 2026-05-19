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

# ROS 2 のインポート
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

"""
all_senser.py

gyairo.py と arduino.py の処理を統合し、スレッドで並行実行。
ArduinoのX, YとジャイロのZ(角度)を合体して出力します。

使い方:
    python3 all_senser.py
"""

# 出力ロック
_output_lock = threading.Lock()

# 最新行を保持
_latest_lines = defaultdict(str)

# 終了フラグ
_should_exit = False

# ジャイロから取得した最新の角度(Z)
_current_gyro_yaw = 0.0
_gyro_yaw_offset = None
_gyro_updated = False

# 自動検出されたポート
_gyro_port = None
_arduino_port = None
_status_message = "Detecting ports..."

def normalize_angle(angle):
    """角度を -180 ~ 180 度に正規化する"""
    while angle > 180:
        angle -= 360
    while angle <= -180:
        angle += 360
    return angle

def auto_detect_ports():
    """自動でジャイロとArduinoのポートを探す"""
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
    if _gyro_port:
        status.append(f"Gyro: {_gyro_port}")
    else:
        status.append(f"Gyro: 未検出(USB)")
        
    if _arduino_port:
        status.append(f"Arduino: {_arduino_port}")
    else:
        status.append(f"Arduino: 未検出(ACM)")
        
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
            # 読み書き権限があるか確認
            if not os.access(dev, os.R_OK | os.W_OK):
                try:
                    # sudo chmod 666 を実行
                    subprocess.run(['sudo', 'chmod', '666', dev], check=True)
                except Exception:
                    pass


def transform_data(data):
    """WT901 IMU データ変換"""
    return struct.unpack('<hhh', data)


def gyairo_thread():
    """gyairo.py の処理をスレッドで実行"""
    global _should_exit, _current_gyro_yaw, _gyro_yaw_offset, _gyro_updated

    baud = 115200
    
    while not _should_exit:
        if not _gyro_port:
            time.sleep(1.0)
            continue
            
        try:
            # 高速通信用にタイムアウトを短くし、バッファを利用する
            ser = serial.Serial(_gyro_port, baud, timeout=0.01)
            buffer = bytearray()
            
            while not _should_exit:
                try:
                    waiting = ser.in_waiting
                    if waiting > 0:
                        buffer.extend(ser.read(waiting))
                        
                        # 1パケットは11バイトなので、11バイト以上ある限り処理を続ける
                        while len(buffer) >= 11:
                            if buffer[0] == 0x55:
                                flag = buffer[1]
                                
                                if flag == 0x53:  # 角度（最新値を保持）
                                    raw_data = buffer[2:8]
                                    ax, ay, az = transform_data(bytes(raw_data))
                                    roll, pitch, yaw = ax/32768*180, ay/32768*180, az/32768*180
                                    
                                    if _gyro_yaw_offset is None:
                                        _gyro_yaw_offset = yaw
                                        
                                    # ジャイロのZ(角度)を初期値を0として更新
                                    _current_gyro_yaw = normalize_angle(yaw - _gyro_yaw_offset)
                                    _gyro_updated = True
                                
                                # 1パケット分（11バイト）削除
                                buffer = buffer[11:]
                            else:
                                # ヘッダが見つからない場合は1バイト進める
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


class OtosOdomNode(Node):
    def __init__(self):
        super().__init__('otos_odom_node')
        self.port = _arduino_port
        self.ser = None
        
        if self.port:
            try:
                self.ser = serial.Serial(self.port, 115200, timeout=0.1)
            except Exception as e:
                with _output_lock:
                    _latest_lines['arduino_err'] = f"[ARDUINO INIT] {e}"

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.last_update = time.time()
        
        # 統合位置計算用の変数
        self.prev_x_raw = None
        self.prev_y_raw = None
        self.prev_theta_arduino = None
        
        # ジャイロの角度を考慮した真の座標
        self.true_x = 0.0
        self.true_y = 0.0

    def update(self):
        if not self.port:
            return
            
        if not self.ser:
            # 再接続を試みる
            try:
                self.ser = serial.Serial(self.port, 115200, timeout=0.1)
                with _output_lock:
                    _latest_lines['arduino_err'] = "" # エラー消去
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
                
                # 初回のみ現在の値を保存して0.0からスタート
                if self.prev_x_raw is None:
                    self.prev_x_raw = x_m_raw
                    self.prev_y_raw = y_m_raw
                    self.prev_theta_arduino = theta_arduino
                
                # Z(ω)はジャイロの値をそのまま使用
                z_deg = _current_gyro_yaw
                z_rad = math.radians(z_deg)
                
                # 1. Arduinoのワールド座標系での移動量（前回からの差分）
                delta_x_ard = x_m_raw - self.prev_x_raw
                delta_y_ard = y_m_raw - self.prev_y_raw
                
                # 2. Arduinoの角度を使って、ロボット本体の「ローカル座標系」での移動量に逆変換
                cos_a = math.cos(self.prev_theta_arduino)
                sin_a = math.sin(self.prev_theta_arduino)
                local_dx = delta_x_ard * cos_a + delta_y_ard * sin_a
                local_dy = -delta_x_ard * sin_a + delta_y_ard * cos_a
                
                # 3. 高精度なジャイロの角度(Z)を使って、真のワールド座標系の移動量に変換
                cos_g = math.cos(z_rad)
                sin_g = math.sin(z_rad)
                true_dx = local_dx * cos_g - local_dy * sin_g
                true_dy = local_dx * sin_g + local_dy * cos_g
                
                # 4. 真の座標を更新
                self.true_x += true_dx
                self.true_y += true_dy
                
                # 次回の計算のために生の値を保存
                self.prev_x_raw = x_m_raw
                self.prev_y_raw = y_m_raw
                self.prev_theta_arduino = theta_arduino

                # ターミナル表示用に一つのX,Y,Zとして合体して表示
                combined_line = f"X: {self.true_x:>6.3f} m, Y: {self.true_y:>6.3f} m, Z: {z_deg:>7.2f} °"
                with _output_lock:
                    _latest_lines['combined'] = combined_line

                # ROS 2 Odometry 配信
                msg = Odometry()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'odom'
                msg.child_frame_id = 'base_link'
                msg.pose.pose.position.x = self.true_x
                msg.pose.pose.position.y = self.true_y
                msg.pose.pose.orientation = self.euler_to_quaternion(0, 0, z_rad)
                self.odom_pub.publish(msg)

            except ValueError as e:
                with _output_lock:
                    _latest_lines['arduino_err'] = f"[PARSE ERROR] {e} | Line: {line}"
            except UnicodeDecodeError as e:
                with _output_lock:
                    _latest_lines['arduino_err'] = f"[DECODE ERROR] {e}"
            except Exception as e:
                with _output_lock:
                    _latest_lines['arduino_err'] = f"[UPDATE ERROR] {e}"
                self.ser = None

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def arduino_thread():
    """arduino.py の処理をスレッドで実行"""
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


def main():
    global _should_exit
    
    # 画面クリア
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

    def _shutdown(signum=None, frame=None):
        global _should_exit
        _should_exit = True
        print('\n\nShutting down...')
        time.sleep(0.5)
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        # メインループ
        sys.stdout.write('\033[2J')  # 全体クリア
        while not _should_exit:
            with _output_lock:
                combined_line = _latest_lines.get('combined', 'Waiting for sensor data...')
                gyro_err = _latest_lines.get('gyro_err', '')
                arduino_err = _latest_lines.get('arduino_err', '')
                
                # 行を固定して表示
                sys.stdout.write('\033[1;0H\033[2K')
                sys.stdout.write(f"[PORT STATUS] {_status_message}\n")
                
                sys.stdout.write('\033[2;0H\033[2K')
                sys.stdout.write(f"[FUSED POSITION] {combined_line}\n")
                
                if gyro_err or arduino_err:
                    sys.stdout.write('\033[3;0H\033[2K')
                    sys.stdout.write(f"{gyro_err} {arduino_err}\n")
                
                sys.stdout.flush()
            
            time.sleep(0.05)
    except KeyboardInterrupt:
        _shutdown()


if __name__ == '__main__':
    main()