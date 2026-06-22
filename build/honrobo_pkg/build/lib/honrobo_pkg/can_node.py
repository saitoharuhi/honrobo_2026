"""
can_node.py — CAN通信ノード

ROS 2トピックとSocketCANバスの橋渡しを行います。
roboware_nodeから受信した速度指令(x, y, ω)を16進数でマイコンへ送信します。

サブスクライブ:
    /can_tx (std_msgs/Int32MultiArray) — CAN送信データ [CAN_ID, byte0, byte1, ...]

SocketCAN (can0) を使用します。事前に setup_can.sh でインターフェースを起動してください。
"""

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import can
import threading
import struct
import sys
import time
from datetime import datetime
import signal
import subprocess


# ============================================================
# CAN信号定義 — マイコンとの通信プロトコル
# ============================================================
CAN_SIGNALS = {
    0x500: {
        'name': 'Buttons_Shapes_Arrows',
        'signals': {
            'Circle':   {'start_byte': 0, 'length': 1, 'type': 'uint8'},
            'Triangle': {'start_byte': 1, 'length': 1, 'type': 'uint8'},
            'Square':   {'start_byte': 2, 'length': 1, 'type': 'uint8'},
            'Cross':    {'start_byte': 3, 'length': 1, 'type': 'uint8'},
            'Up':       {'start_byte': 4, 'length': 1, 'type': 'uint8'},
            'Down':     {'start_byte': 5, 'length': 1, 'type': 'uint8'},
            'Left':     {'start_byte': 6, 'length': 1, 'type': 'uint8'},
            'Right':    {'start_byte': 7, 'length': 1, 'type': 'uint8'},
        }
    },
    0x501: {
        'name': 'Buttons_LR',
        'signals': {
            'R1':     {'start_byte': 0, 'length': 1, 'type': 'uint8'},
            'R2_Btn': {'start_byte': 1, 'length': 1, 'type': 'uint8'},
            'R3':     {'start_byte': 2, 'length': 1, 'type': 'uint8'},
            'L1':     {'start_byte': 3, 'length': 1, 'type': 'uint8'},
            'L2_Btn': {'start_byte': 4, 'length': 1, 'type': 'uint8'},
            'L3':     {'start_byte': 5, 'length': 1, 'type': 'uint8'},
        }
    },
    0x502: {
        'name': 'Buttons_System',
        'signals': {
            'Share':   {'start_byte': 0, 'length': 1, 'type': 'uint8'},
            'Options': {'start_byte': 1, 'length': 1, 'type': 'uint8'},
            'Home_PS': {'start_byte': 2, 'length': 1, 'type': 'uint8'},
        }
    },
    0x510: {
        'name': 'Movement_XYW',
        'signals': {
            'VX': {'start_byte': 0, 'length': 2, 'type': 'int16', 'unit': 'mm/s'},
            'VY': {'start_byte': 2, 'length': 2, 'type': 'int16', 'unit': 'mm/s'},
            'VW': {'start_byte': 4, 'length': 2, 'type': 'int16', 'unit': 'deg/s*10'},
        }
    }
}


class CanNode(Node):
    """SocketCAN ↔ ROS 2 ブリッジノード"""

    def __init__(self):
        super().__init__('can_node')

        self.bus = None
        self.running = True

        # 表示用データ
        self.state = {
            'rx': {},
            'tx': 'None',
            'status': 'Connecting...',
            'error': 'None',
            'rx_count': 0,
            'heartbeat': 0,
            'version': can.__version__,
        }

        # 既知IDを事前登録
        for arbid in CAN_SIGNALS:
            self.state['rx'][arbid] = {
                'ts': '--:--:--', 'dlc': 0, 'data': '--', 'count': 0,
                'decoded': {
                    'name': CAN_SIGNALS[arbid]['name'],
                    'signals': {},
                }
            }

        self.seen_ids = set()
        self.display_lock = threading.Lock()

        # CAN読み取りタイマー (100Hz)
        self.create_timer(0.01, self._can_reader_timer)
        # 画面表示タイマー (20Hz)
        self.create_timer(0.05, self._print_display)
        # ステータス確認タイマー (1Hz)
        self.create_timer(1.0, self._status_callback)

        # CAN送信用サブスクライバー (roboware_nodeから受信)
        self.tx_sub = self.create_subscription(
            Int32MultiArray, 'can_tx', self._tx_callback, 10
        )

    # ─── CAN送信 ───────────────────────────────
    def _tx_callback(self, msg):
        """roboware_nodeからのデータをCANフレームとして送信"""
        if not self.bus or len(msg.data) < 1:
            return

        can_id = msg.data[0]
        data_bytes = list(msg.data[1:])

        try:
            if len(data_bytes) > 8:
                self.get_logger().error(
                    f"CAN data too long: {len(data_bytes)}B → 8Bに切り詰め"
                )
                data_bytes = data_bytes[:8]

            can_msg = can.Message(
                arbitration_id=can_id,
                data=bytes(data_bytes),
                is_extended_id=False,
            )

            if self.bus and self.running:
                self.bus.send(can_msg)

            with self.display_lock:
                hex_str = " ".join([f"{b:02X}" for b in data_bytes])
                self.state['tx'] = f"ID:0x{can_id:03X} Data:[{hex_str}]"

        except Exception as e:
            self.get_logger().error(f"TX Error: {e}")
            with self.display_lock:
                self.state['tx'] = f"Error: {e}"

    # ─── CAN接続 ───────────────────────────────
    def _setup_can_bus(self):
        """SocketCAN (can0) への接続を試みる"""
        try:
            self.get_logger().info("Connecting to can0...")
            self.bus = can.Bus(
                channel='can0',
                interface='socketcan',
                receive_own_messages=True,
            )
            with self.display_lock:
                self.state['status'] = "Connected (can0)"
            return True
        except Exception as e:
            with self.display_lock:
                self.state['status'] = f"Error: {e}"
            return False

    # ─── CAN受信タイマー ───────────────────────
    def _can_reader_timer(self):
        """非ブロッキングでCANメッセージを一括読み出し"""
        if not self.bus:
            if not self._setup_can_bus():
                return

        msgs = []
        while True:
            try:
                m = self.bus.recv(timeout=0.0)
                if not m:
                    break
                msgs.append(m)
            except Exception as e:
                with self.display_lock:
                    self.state['error'] = f"Recv Error: {e}"
                break

        if not msgs:
            return

        with self.display_lock:
            for message in msgs:
                self.state['rx_count'] += 1
                arbid = message.arbitration_id
                data_hex = " ".join([f"{b:02X}" for b in message.data])
                decoded = self._decode_signals(arbid, message.data)
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]

                if arbid not in self.state['rx']:
                    self.state['rx'][arbid] = {'count': 0}

                self.state['rx'][arbid].update({
                    'ts': ts, 'dlc': message.dlc,
                    'data': data_hex, 'decoded': decoded,
                    'count': self.state['rx'][arbid]['count'] + 1,
                })

                if arbid not in self.seen_ids:
                    self.get_logger().info(f"First RX from ID: 0x{arbid:03X}")
                    self.seen_ids.add(arbid)

    # ─── 信号デコード ──────────────────────────
    def _decode_signals(self, arbid, data):
        """CANメッセージを信号定義に基づいてデコード"""
        if arbid not in CAN_SIGNALS:
            return None

        msg_def = CAN_SIGNALS[arbid]
        decoded = {'name': msg_def['name'], 'signals': {}}

        try:
            for sig_name, sig_info in msg_def['signals'].items():
                sb = sig_info['start_byte']
                length = sig_info['length']
                sig_type = sig_info['type']
                unit = sig_info.get('unit', '')

                byte_data = data[sb:sb + length]
                if len(byte_data) < length:
                    continue

                if sig_type == 'int16':
                    value = struct.unpack('>h', byte_data)[0]
                elif sig_type == 'uint16':
                    value = struct.unpack('>H', byte_data)[0]
                elif sig_type == 'uint8':
                    value = byte_data[0]
                elif sig_type == 'int8':
                    value = struct.unpack('b', byte_data)[0]
                else:
                    value = byte_data.hex()

                decoded['signals'][sig_name] = {
                    'value': value, 'unit': unit
                }
        except Exception as e:
            self.get_logger().error(f"Decode error ({sig_name}): {e}")

        return decoded

    # ─── ターミナル表示 ────────────────────────
    def _print_display(self):
        """ターミナルにCAN通信状況をリアルタイム表示"""
        try:
            with self.display_lock:
                self.state['heartbeat'] += 1
                blink = "●" if (self.state['heartbeat'] // 5) % 2 == 0 else " "

                sys.stdout.write('\033[2J\033[H')
                sys.stdout.write(
                    "=" * 80 + "\n"
                    f" CAN NODE [{blink}] | "
                    f"Status: {self.state['status']} | "
                    f"RX: {self.state['rx_count']} | "
                    f"lib-can: v{self.state['version']}\n"
                )
                if self.state['error'] != 'None':
                    sys.stdout.write(
                        f" ERROR: {self.state['error']}\n"
                    )
                sys.stdout.write("=" * 80 + "\n")

                sys.stdout.write(f"[LAST TX] {self.state['tx']}\n")
                sys.stdout.write("-" * 80 + "\n")

                sys.stdout.write(
                    f"{'TIME':<12} | {'CAN ID':<6} | "
                    f"{'CNT':<4} | {'DATA':<23} | DECODED\n"
                )
                sys.stdout.write("-" * 100 + "\n")

                for arbid in sorted(self.state['rx'].keys()):
                    info = self.state['rx'][arbid]
                    name = (
                        info['decoded']['name']
                        if info.get('decoded') else "Unknown"
                    )

                    decoded_str = ""
                    if info.get('decoded') and info['decoded'].get('signals'):
                        parts = [
                            f"{n}:{d['value']}"
                            for n, d in info['decoded']['signals'].items()
                        ]
                        decoded_str = " -> " + ", ".join(parts)

                    sys.stdout.write(
                        f"{info.get('ts', '--'):<12} | "
                        f"0x{arbid:03X}  | "
                        f"{info['count']:<4} | "
                        f"{info.get('data', '--'):<23} | "
                        f"{name}{decoded_str}\n"
                    )

                sys.stdout.write("=" * 80 + "\n")
                sys.stdout.flush()
        except Exception as e:
            self.get_logger().error(f"Display error: {e}")

    def _status_callback(self):
        """定期ステータス確認"""
        if self.bus:
            try:
                if self.bus.state.name == 'ACTIVE':
                    self.get_logger().debug("CAN bus active")
            except Exception:
                pass

    def destroy_node(self):
        self.running = False
        if self.bus:
            self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CanNode()

    def _shutdown_prompt(signum=None, frame=None):
        # 一時的にSIGINTを無視して多重割り込み防止
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        
        # 描画と被らないように改行を入れて表示
        sys.stdout.write(f"\n\033[1;33m[Ctrl+C を検知しました: can_node]\033[0m\n")
        sys.stdout.flush()
        
        while True:
            try:
                # 標準入力バッファクリア
                try:
                    import termios
                    termios.tcflush(sys.stdin, termios.TCIFLUSH)
                except Exception:
                    pass
                ans = input("すべてのプログラムを終了しますか？ (a: すべて終了 / y: このプログラムのみ終了 / c: キャンセルして再開): ").strip().lower()
            except (KeyboardInterrupt, EOFError):
                ans = 'y'
            
            if ans == 'a':
                print("すべてのプログラムを終了しています...")
                subprocess.run(["bash", "/home/haru/Documents/honrobo_2026/scripts/stop_all.sh"])
                node.destroy_node()
                rclpy.shutdown()
                sys.exit(0)
            elif ans == 'y':
                print("can_node を終了します...")
                node.destroy_node()
                rclpy.shutdown()
                sys.exit(0)
            elif ans == 'c':
                print("実行を再開します...")
                signal.signal(signal.SIGINT, _shutdown_prompt)
                return
            else:
                print("無効な入力です。'a', 'y', 'c' のいずれかを入力してください。")

    signal.signal(signal.SIGINT, _shutdown_prompt)

    try:
        while rclpy.ok():
            try:
                rclpy.spin(node)
                break
            except KeyboardInterrupt:
                # _shutdown_promptが先に実行され、c (キャンセル) の場合のみここに来る
                pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
