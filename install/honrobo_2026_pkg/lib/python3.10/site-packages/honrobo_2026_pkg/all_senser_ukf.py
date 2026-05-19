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
import numpy as np
import can

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

# ==========================================
# 状態・グローバル変数
# ==========================================
_output_lock = threading.Lock()
_should_exit = False
_status_message = "Detecting ports..."
_error_messages = {"gyro": "", "arduino": "", "can": "", "ukf": ""}

_gyro_port = None
_arduino_port = None

# センサー最新値 (補正済み絶対座標)
_latest_gyro_yaw = 0.0
_gyro_active = False
_gyro_offset = None

_latest_otos_x, _latest_otos_y = 0.0, 0.0
_otos_active = False
_otos_offset_x, _otos_offset_y = None, None

_latest_wheel_x, _latest_wheel_y, _latest_wheel_yaw = 0.0, 0.0, 0.0
_wheel_active = False
_wheel_offset_x, _wheel_offset_y, _wheel_offset_yaw = None, None, None

# ==========================================
# UKF (アンセンテッド・カルマンフィルタ) の実装
# ==========================================
def normalize_angle(angle):
    while angle > math.pi: angle -= 2 * math.pi
    while angle <= -math.pi: angle += 2 * math.pi
    return angle

def average_angles(angles, weights):
    sum_sin = np.sum(weights * np.sin(angles))
    sum_cos = np.sum(weights * np.cos(angles))
    return np.arctan2(sum_sin, sum_cos)

class SimpleUKF:
    def __init__(self):
        self.dim_x = 3 # State: [X, Y, Yaw]
        self.x = np.zeros(self.dim_x)
        self.P = np.eye(self.dim_x) * 0.1
        self.Q = np.diag([0.001, 0.001, 0.001]) # プロセスノイズ (1ループあたりの不確実性)
        self.kappa = 0.0
        
        # --- 信頼度の設定 (分散 R を調整して重みを決定) ---
        # X, Y: OTOS 80%, Wheel 20%  => Wheelの分散をOTOSの4倍に設定
        self.R_otos = np.diag([0.01, 0.01])
        self.R_wheel_xy = np.diag([0.04, 0.04])
        
        # Yaw: Gyro 40%, Wheel 60%  => Gyroの分散をWheelの1.5倍に設定
        self.R_gyro = np.array([[0.015]])
        self.R_wheel_yaw = np.array([[0.010]])

    def get_sigma_points(self):
        n = self.dim_x
        sigmas = np.zeros((2 * n + 1, n))
        sigmas[0] = self.x
        # 数値誤差回避のため微小値を足す
        U = np.linalg.cholesky((n + self.kappa) * self.P + np.eye(n)*1e-9)
        for i in range(n):
            sigmas[i + 1] = self.x + U[i]
            sigmas[n + i + 1] = self.x - U[i]
            # 角度(Yaw)の正規化
            sigmas[i + 1][2] = normalize_angle(sigmas[i + 1][2])
            sigmas[n + i + 1][2] = normalize_angle(sigmas[n + i + 1][2])
        return sigmas

    def get_weights(self):
        n = self.dim_x
        Wm = np.full(2 * n + 1, 1.0 / (2 * (n + self.kappa)))
        Wm[0] = self.kappa / (n + self.kappa)
        return Wm, Wm.copy()

    def predict(self):
        sigmas = self.get_sigma_points()
        Wm, Wc = self.get_weights()
        
        # f(x) = x (今回は移動モデルなしのランダムウォークとして予測)
        sigmas_f = sigmas
        
        self.x[0] = np.sum(Wm * sigmas_f[:, 0])
        self.x[1] = np.sum(Wm * sigmas_f[:, 1])
        self.x[2] = average_angles(sigmas_f[:, 2], Wm)
        
        y = sigmas_f - self.x
        y[:, 2] = np.array([normalize_angle(a) for a in y[:, 2]])
        
        P = np.zeros((self.dim_x, self.dim_x))
        for i in range(2 * self.dim_x + 1):
            y_i = y[i][:, np.newaxis]
            P += Wc[i] * np.dot(y_i, y_i.T)
        self.P = P + self.Q

    def update(self, z, R, H_func, is_angle=False):
        sigmas = self.get_sigma_points()
        Wm, Wc = self.get_weights()
        
        # シグマポイントを観測空間に変換
        sigmas_h = np.array([H_func(s) for s in sigmas])
        z_dim = len(z)
        
        if is_angle:
            zp = np.array([average_angles(sigmas_h[:, 0], Wm)])
            y = sigmas_h - zp
            y[:, 0] = np.array([normalize_angle(a) for a in y[:, 0]])
        else:
            zp = np.zeros(z_dim)
            for i in range(z_dim):
                zp[i] = np.sum(Wm * sigmas_h[:, i])
            y = sigmas_h - zp
            
        x_diff = sigmas - self.x
        x_diff[:, 2] = np.array([normalize_angle(a) for a in x_diff[:, 2]])
        
        S = np.zeros((z_dim, z_dim))
        C = np.zeros((self.dim_x, z_dim))
        for i in range(2 * self.dim_x + 1):
            y_i = y[i][:, np.newaxis]
            x_i = x_diff[i][:, np.newaxis]
            S += Wc[i] * np.dot(y_i, y_i.T)
            C += Wc[i] * np.dot(x_i, y_i.T)
        S += R
        
        # カルマンゲイン
        try:
            K = np.dot(C, np.linalg.inv(S))
        except np.linalg.LinAlgError:
            return # 逆行列が計算できない場合はスキップ
        
        innov = z - zp
        if is_angle:
            innov[0] = normalize_angle(innov[0])
            
        self.x = self.x + np.dot(K, innov)
        self.x[2] = normalize_angle(self.x[2])
        self.P = self.P - np.dot(K, np.dot(S, K.T))

# ==========================================
# デバイス自動認識と通信スレッド
# ==========================================
def auto_detect_ports():
    global _gyro_port, _arduino_port, _status_message
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if 'USB' in p.device and _gyro_port is None:
            _gyro_port = p.device
        elif 'ACM' in p.device and _arduino_port is None:
            _arduino_port = p.device
    _status_message = f"Gyro: {_gyro_port or '未検出'} | Arduino: {_arduino_port or '未検出'} | CAN: can0"

def setup_permissions():
    for dev in [_gyro_port, _arduino_port]:
        if dev and os.path.exists(dev) and not os.access(dev, os.R_OK | os.W_OK):
            try: subprocess.run(['sudo', 'chmod', '666', dev], check=True)
            except: pass

def gyairo_thread():
    global _should_exit, _latest_gyro_yaw, _gyro_active, _gyro_offset, _error_messages
    while not _should_exit:
        if not _gyro_port: time.sleep(1.0); continue
        try:
            ser = serial.Serial(_gyro_port, 115200, timeout=0.01)
            buffer = bytearray()
            _error_messages["gyro"] = ""
            while not _should_exit:
                if ser.in_waiting > 0:
                    buffer.extend(ser.read(ser.in_waiting))
                    while len(buffer) >= 11:
                        if buffer[0] == 0x55 and buffer[1] == 0x53:
                            ax, ay, az = struct.unpack('<hhh', buffer[2:8])
                            yaw = az / 32768 * 180
                            if _gyro_offset is None: _gyro_offset = yaw
                            _latest_gyro_yaw = normalize_angle(math.radians(yaw - _gyro_offset))
                            _gyro_active = True
                            buffer = buffer[11:]
                        else:
                            buffer.pop(0)
                else: time.sleep(0.001)
        except Exception as e:
            _error_messages["gyro"] = f"[GYRO ERR] {e}"
            _gyro_active = False
            time.sleep(1.0)

def arduino_thread():
    global _should_exit, _latest_otos_x, _latest_otos_y, _otos_active, _otos_offset_x, _otos_offset_y, _error_messages
    while not _should_exit:
        if not _arduino_port: time.sleep(1.0); continue
        try:
            ser = serial.Serial(_arduino_port, 115200, timeout=0.1)
            _error_messages["arduino"] = ""
            while not _should_exit:
                line = ser.readline().decode('utf-8').strip()
                if line and not line.startswith('#'):
                    x_inch, y_inch, head_deg = map(float, line.split(','))
                    x_m = x_inch * 0.0254
                    y_m = y_inch * 0.0254
                    
                    if _otos_offset_x is None:
                        _otos_offset_x, _otos_offset_y = x_m, y_m
                        
                    _latest_otos_x = x_m - _otos_offset_x
                    _latest_otos_y = y_m - _otos_offset_y
                    _otos_active = True
        except Exception as e:
            _error_messages["arduino"] = f"[OTOS ERR] {e}"
            _otos_active = False
            time.sleep(1.0)

def can_thread():
    """ 計測輪のデータを受信 (CAN ID: 0x520 を想定)
        データ形式: [X(int16), Y(int16), Yaw(int16)] (mm, mm, deg)
    """
    global _should_exit, _latest_wheel_x, _latest_wheel_y, _latest_wheel_yaw
    global _wheel_active, _wheel_offset_x, _wheel_offset_y, _wheel_offset_yaw, _error_messages
    
    while not _should_exit:
        try:
            bus = can.interface.Bus(channel='can0', bustype='socketcan')
            _error_messages["can"] = ""
            while not _should_exit:
                msg = bus.recv(0.1)
                if msg and msg.arbitration_id == 0x520 and msg.dlc >= 6:
                    x_mm, y_mm, yaw_deg = struct.unpack('<hhh', msg.data[:6])
                    x_m = x_mm / 1000.0
                    y_m = y_mm / 1000.0
                    yaw_rad = math.radians(yaw_deg)
                    
                    if _wheel_offset_x is None:
                        _wheel_offset_x, _wheel_offset_y, _wheel_offset_yaw = x_m, y_m, yaw_rad
                        
                    _latest_wheel_x = x_m - _wheel_offset_x
                    _latest_wheel_y = y_m - _wheel_offset_y
                    _latest_wheel_yaw = normalize_angle(yaw_rad - _wheel_offset_yaw)
                    _wheel_active = True
        except Exception as e:
            _error_messages["can"] = f"[CAN ERR] {e}"
            _wheel_active = False
            time.sleep(1.0)

# ==========================================
# ROS 2 ノード (UKFの実行と配信)
# ==========================================
class UkfOdomNode(Node):
    def __init__(self):
        super().__init__('ukf_odom_node')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.ukf = SimpleUKF()
        # 50HzでUKFを更新・配信
        self.timer = self.create_timer(0.02, self.update)
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def update(self):
        try:
            self.ukf.predict()
            
            # OTOS の X, Y (80% 信用)
            if _otos_active:
                z_otos = np.array([_latest_otos_x, _latest_otos_y])
                self.ukf.update(z_otos, self.ukf.R_otos, lambda x: x[:2])
                
            # 計測輪 の X, Y (20% 信用) と Yaw (60% 信用)
            if _wheel_active:
                z_wheel_xy = np.array([_latest_wheel_x, _latest_wheel_y])
                self.ukf.update(z_wheel_xy, self.ukf.R_wheel_xy, lambda x: x[:2])
                
                z_wheel_yaw = np.array([_latest_wheel_yaw])
                self.ukf.update(z_wheel_yaw, self.ukf.R_wheel_yaw, lambda x: np.array([x[2]]), is_angle=True)
                
            # ジャイロ の Yaw (40% 信用)
            if _gyro_active:
                z_gyro = np.array([_latest_gyro_yaw])
                self.ukf.update(z_gyro, self.ukf.R_gyro, lambda x: np.array([x[2]]), is_angle=True)

            fused_x, fused_y, fused_yaw = self.ukf.x
            
            # ターミナル表示用に文字列を格納
            with _output_lock:
                _error_messages["ukf"] = f"UKF: X:{fused_x:>6.3f} Y:{fused_y:>6.3f} Yaw:{math.degrees(fused_yaw):>6.2f}°"

            # ROS 2 Odometry 配信
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'
            msg.pose.pose.position.x = fused_x
            msg.pose.pose.position.y = fused_y
            msg.pose.pose.orientation = self.euler_to_quaternion(0, 0, fused_yaw)
            self.odom_pub.publish(msg)
        except Exception as e:
            with _output_lock:
                _error_messages["ukf"] = f"[UKF ERR] {e}"

def ros_thread():
    rclpy.init()
    node = UkfOdomNode()
    while not _should_exit and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

def main():
    global _should_exit
    sys.stdout.write('\033[2J\033[H'); sys.stdout.flush()
    print("Initializing UKF Localization...")
    
    auto_detect_ports()
    setup_permissions()

    threading.Thread(target=gyairo_thread, daemon=True).start()
    threading.Thread(target=arduino_thread, daemon=True).start()
    threading.Thread(target=can_thread, daemon=True).start()
    threading.Thread(target=ros_thread, daemon=True).start()

    def _shutdown(signum=None, frame=None):
        global _should_exit
        _should_exit = True
        print('\nShutting down...')
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        sys.stdout.write('\033[2J')
        while not _should_exit:
            with _output_lock:
                sys.stdout.write('\033[1;0H\033[2K')
                sys.stdout.write(f"[STATUS] {_status_message}\n")
                
                sys.stdout.write('\033[2;0H\033[2K')
                sys.stdout.write(f"{_error_messages.get('ukf', 'Waiting for sensors...')}\n")
                
                sys.stdout.write('\033[3;0H\033[2K')
                errs = " ".join([v for k,v in _error_messages.items() if k != "ukf" and v])
                if errs: sys.stdout.write(f"{errs}\n")
                sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        _shutdown()

if __name__ == '__main__':
    main()
