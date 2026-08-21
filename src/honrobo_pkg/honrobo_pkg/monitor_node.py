#!/usr/bin/env python3
"""
monitor_node.py — 操縦者用リアルタイム・ロボット状態モニターノード

ロボットPCから配信されるROS 2トピックを受信し、
自己位置(x, y, yaw)、自動/手動モード、最新のCAN送信値を
操縦者側のターミナル上にリアルタイムで一括表示します。
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray, Bool, String
import sys
import math
import struct
import time
from datetime import datetime

class OperatorMonitorNode(Node):
    def __init__(self):
        super().__init__('operator_monitor_node')

        # 状態保持用変数
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.odom_count = 0
        self.last_odom_time = 0.0

        self.auto_mode = False
        self.mode_str = "MANUAL"

        self.last_can_id = 0
        self.last_can_bytes = []
        self.last_can_time = 0.0
        self.decoded_vel = {"vx": 0.0, "vy": 0.0, "vw": 0.0}
        self.decoded_yaw = 0.0

        self.robot_ip = "Unknown (Waiting for robot...)"

        # サブスクライバ設定
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)
        self.create_subscription(Bool, 'auto_mode', self._mode_cb, 10)
        self.create_subscription(Int32MultiArray, 'can_tx', self._can_tx_cb, 10)
        self.create_subscription(String, 'robot_ip', self._ip_cb, 10)

        # 画面更新タイマー (10Hz / 0.1s周期)
        self.create_timer(0.1, self._render_screen)

        # 画面クリア
        sys.stdout.write("\033[2J")
        sys.stdout.flush()

    def _odom_cb(self, msg: Odometry):
        self.odom_count += 1
        self.last_odom_time = time.time()
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        
        # クォータニオンからYaw(ヨー角)への変換
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.pose_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _mode_cb(self, msg: Bool):
        self.auto_mode = msg.data
        self.mode_str = "AUTO" if msg.data else "MANUAL"

    def _ip_cb(self, msg: String):
        self.robot_ip = msg.data

    def _can_tx_cb(self, msg: Int32MultiArray):
        if len(msg.data) < 1:
            return
        
        self.last_can_id = msg.data[0]
        self.last_can_bytes = list(msg.data[1:])
        self.last_can_time = time.time()

        # 速度指令 ID: 0x510 の場合はデコードする
        if self.last_can_id == 0x510 and len(self.last_can_bytes) >= 6:
            try:
                b_data = bytes(self.last_can_bytes[:6])
                vx, vy, vw = struct.unpack('>hhh', b_data)
                # VEL_SCALE = 10.0 で割って実数値に戻す
                self.decoded_vel["vx"] = vx / 10.0
                self.decoded_vel["vy"] = vy / 10.0
                self.decoded_vel["vw"] = vw / 10.0
            except Exception:
                pass
        # 自己位置Yaw ID: 0x520 の場合はデコードする (度数法の整数, int16型2B)
        elif self.last_can_id == 0x520 and len(self.last_can_bytes) >= 2:
            try:
                b_data = bytes(self.last_can_bytes[:2])
                yaw_val = struct.unpack('>h', b_data)[0]
                self.decoded_yaw = float(yaw_val)
            except Exception:
                pass

    def _render_screen(self):
        # 画面を上書き（カーソルを左上へ）
        sys.stdout.write('\033[H')

        now_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        # 通信状態の判定
        odom_status = "\033[1;32m[ OK ]\033[0m"
        if time.time() - self.last_odom_time > 1.0:
            odom_status = "\033[1;33m[STALE]\033[0m" if self.odom_count > 0 else "\033[1;31m[NO DATA]\033[0m"

        can_status = "\033[1;32m[ OK ]\033[0m"
        if time.time() - self.last_can_time > 1.0:
            can_status = "\033[1;33m[STALE]\033[0m" if self.last_can_id > 0 else "\033[1;31m[NO DATA]\033[0m"

        buf = []
        buf.append("====================================================")
        buf.append(f"  🤖 HONROBO 2026 OPERATOR MONITOR  |  Time: {now_str}")
        buf.append("====================================================")
        
        # モード表示
        if self.auto_mode:
            buf.append(f"  [MODE]      \033[1;36m{self.mode_str:<10}\033[0m (Autonomous Navigation)")
        else:
            buf.append(f"  [MODE]      \033[1;32m{self.mode_str:<10}\033[0m (Manual Controller)")
            
        buf.append("----------------------------------------------------")
        buf.append(f"  [ROBOT IP]  {self.robot_ip}")
        buf.append(f"    Web UI  :  http://{self.robot_ip}:8080")
        buf.append(f"    WebSocket:  ws://{self.robot_ip}:8765")
        buf.append("----------------------------------------------------")
        
        # 自己位置表示
        buf.append(f"  [ODOMETRY]  Status: {odom_status} (Rx: {self.odom_count})")
        buf.append(f"    X (Right) :  {self.pose_x:>6.3f} m")
        buf.append(f"    Y (Forward):  {self.pose_y:>6.3f} m")
        buf.append(f"    Yaw (Angle):  {math.degrees(self.pose_yaw):>6.1f} deg")
        
        buf.append("----------------------------------------------------")
        
        # CAN送信値表示
        can_hex_str = " ".join([f"{b:02X}" for b in self.last_can_bytes])
        buf.append(f"  [CAN TX]    Status: {can_status}")
        buf.append(f"    Last ID   :  0x{self.last_can_id:03X}")
        buf.append(f"    Raw Bytes :  [{can_hex_str}]")
        
        # 速度指令の場合の詳細表示
        if self.last_can_id == 0x510:
            buf.append(f"    Velocity Command Decoded:")
            buf.append(f"      Vx (Lat)   : {self.decoded_vel['vx']:>6.1f} mm/s")
            buf.append(f"      Vy (Fwd)   : {self.decoded_vel['vy']:>6.1f} mm/s")
            buf.append(f"      Vw (Rotate): {self.decoded_vel['vw']:>6.1f} deg/s")
        elif self.last_can_id == 0x520:
            buf.append(f"    Yaw Angle Decoded:")
            buf.append(f"      Yaw (Deg)  : {self.decoded_yaw:>6.0f} deg")
            buf.append("")
            buf.append("")
        else:
            buf.append("    (No active velocity/yaw payload to decode)")
            buf.append("")
            buf.append("")
            
        buf.append("====================================================")
        buf.append("\033[K") # 行末までクリア

        sys.stdout.write("\n".join(buf) + "\n")
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = OperatorMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
