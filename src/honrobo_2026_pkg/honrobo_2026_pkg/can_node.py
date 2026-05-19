import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import can
import threading
from datetime import datetime
import struct
import sys
import time


# ============================================================================
# CAN信号定義
# STM32から送られてくるCANメッセージを定義します
# ============================================================================
CAN_SIGNALS = {
    0x500: {
        'name': 'Buttons_Shapes_Arrows',
        'signals': {
            'Circle': {'start_byte': 0, 'length': 1, 'type': 'uint8'},
            'Triangle': {'start_byte': 1, 'length': 1, 'type': 'uint8'},
            'Square': {'start_byte': 2, 'length': 1, 'type': 'uint8'},
            'Cross': {'start_byte': 3, 'length': 1, 'type': 'uint8'},
            'Up': {'start_byte': 4, 'length': 1, 'type': 'uint8'},
            'Down': {'start_byte': 5, 'length': 1, 'type': 'uint8'},
            'Left': {'start_byte': 6, 'length': 1, 'type': 'uint8'},
            'Right': {'start_byte': 7, 'length': 1, 'type': 'uint8'},
        }
    },
    0x501: {
        'name': 'Buttons_LR',
        'signals': {
            'R1': {'start_byte': 0, 'length': 1, 'type': 'uint8'},
            'R2_Btn': {'start_byte': 1, 'length': 1, 'type': 'uint8'},
            'R3': {'start_byte': 2, 'length': 1, 'type': 'uint8'},
            'L1': {'start_byte': 3, 'length': 1, 'type': 'uint8'},
            'L2_Btn': {'start_byte': 4, 'length': 1, 'type': 'uint8'},
            'L3': {'start_byte': 5, 'length': 1, 'type': 'uint8'},
        }
    },
    0x502: {
        'name': 'Buttons_System',
        'signals': {
            'Share': {'start_byte': 0, 'length': 1, 'type': 'uint8'},
            'Options': {'start_byte': 1, 'length': 1, 'type': 'uint8'},
            'Home_PS': {'start_byte': 2, 'length': 1, 'type': 'uint8'},
        }
    },
    0x510: {
        'name': 'Movement_Sticks',
        'signals': {
            'VX': {'start_byte': 0, 'length': 2, 'type': 'int16', 'unit': 'mm/s'},
            'VY': {'start_byte': 2, 'length': 2, 'type': 'int16', 'unit': 'mm/s'},
            'VZ': {'start_byte': 4, 'length': 2, 'type': 'int16', 'unit': 'deg/s'},
        }
    }
}


class MyCanNode(Node):
    def __init__(self):
        super().__init__('can_node')
        
        # CAN通信の設定
        # USB-CANモジュール用のインターフェース設定
        # 一般的なUSB-CANアダプター（PEAK PCAN, Vector CANoe等）に対応
        self.bus = None
        self.reader_thread = None
        self.running = True
        
        # 表示用データ
        self.state = {
            'rx': {},
            'tx': 'None',
            'status': 'Connecting...',
            'error': 'None',
            'rx_count': 0,
            'loop_count': 0,
            'heartbeat': 0,
            'version': can.__version__
        }
        
        # 既知のIDをあらかじめ登録して見やすくする
        for arbid in CAN_SIGNALS.keys():
            self.state['rx'][arbid] = {
                'ts': '--:--:--', 'dlc': 0, 'data': '--', 'count': 0,
                'decoded': {'name': CAN_SIGNALS[arbid]['name'], 'signals': {}}
            }
            
        self.seen_ids = set()
        self.display_lock = threading.Lock()
        self.notifier = None
        
        # デモコードと同様に、タイマーで定期的にバッファから一気に読み出す方式に変更
        self.create_timer(0.01, self.can_reader_timer)
        
        # 周期的に表示を更新 (20Hz)
        self.create_timer(0.05, self.print_display)
        # 周期的にノードの状態を確認
        self.create_timer(1.0, self.status_callback)

        # CAN送信用のサブスクライバー (roboware_nodeから受信)
        self.tx_sub = self.create_subscription(
            Int32MultiArray,
            'can_tx_topic',
            self.tx_callback,
            10
        )

    def tx_callback(self, msg):
        """roboware_nodeからのデータをCANとして送信"""
        if not self.bus:
            return
            
        if len(msg.data) < 1:
            return
            
        can_id = msg.data[0]
        # msg.dataはarray.array('i')のため、そのままbytes()に渡すと4バイト/要素になってしまう！
        # 必ず一度list()に変換してからbytes()に渡すことで正しいデータ長(1バイト/要素)にする
        data_bytes = list(msg.data[1:])
        try:
            # データの長さをチェック (標準CANは最大8バイト)
            if len(data_bytes) > 8:
                self.get_logger().error(f"CAN data too long: {len(data_bytes)} bytes. Truncating to 8.")
                data_bytes = data_bytes[:8]

            can_msg = can.Message(
                arbitration_id=can_id,
                data=bytes(data_bytes), # 明示的にbytes型に変換
                is_extended_id=False
            )
            # バスが有効な場合のみ送信
            if self.bus and self.running:
                self.bus.send(can_msg)
            
            with self.display_lock:
                data_hex = " ".join([f"{b:02X}" for b in data_bytes])
                self.state['tx'] = f"ID:0x{can_id:03X} Data:[{data_hex}]"
                
        except Exception as e:
            self.get_logger().error(f"TX Error: {e}")
            with self.display_lock:
                self.state['tx'] = f"Error: {e}"

    def setup_can_bus(self):
        """
        USB-CANモジュールをセットアップ
        can0 (SocketCAN) で接続します
        """
        try:
            self.get_logger().info("Attempting to connect to can0...")
            
            # デモコードと同様に標準の Bus を使用し、
            # 自分が送信したデータも受信画面（RX）に表示させるためにループバックを有効化
            self.bus = can.Bus(
                channel='can0',
                interface='socketcan',
                receive_own_messages=True
            )
            with self.display_lock:
                self.state['status'] = f"Connected (can0)"
            return True
            
        except Exception as e:
            with self.display_lock:
                self.state['status'] = f"Error: {e}"
            return False

    def can_reader_timer(self):
        """
        デモコードを参考にした、非ブロッキングの一括読み出しループ
        """
        if not self.bus:
            if not self.setup_can_bus():
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
            self.state['loop_count'] += 1
            for message in msgs:
                self.state['rx_count'] += 1
                arbid = message.arbitration_id
                data_hex = " ".join([f"{b:02X}" for b in message.data])
                decoded = self.decode_can_signals(arbid, message.data)
                
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                
                if arbid not in self.state['rx']:
                    self.state['rx'][arbid] = {'count': 0}
                
                self.state['rx'][arbid].update({
                    'ts': ts,
                    'dlc': message.dlc,
                    'data': data_hex,
                    'decoded': decoded,
                    'count': self.state['rx'][arbid]['count'] + 1
                })
                
                # 初めて見るIDの場合はログにも残す
                if arbid not in self.seen_ids:
                    self.get_logger().info(f"First RX from ID: 0x{arbid:03X}")
                    self.seen_ids.add(arbid)

    def print_display(self):
        try:
            with self.display_lock:
                self.state['heartbeat'] += 1
                blink = "*" if (self.state['heartbeat'] // 5) % 2 == 0 else " "
                
                # 画面クリア
                sys.stdout.write('\033[2J\033[H')
                
                sys.stdout.write("================================================================================\n")
                sys.stdout.write(f" CAN NODE [{blink}] | Status: {self.state['status']} | RX Total: {self.state['rx_count']} | lib-can: v{self.state['version']}\n")
                if self.state['error'] != 'None':
                    sys.stdout.write(f" INTERNAL ERROR: {self.state['error']}\n")
                sys.stdout.write("================================================================================\n")
                
                # 送信ステータス
                sys.stdout.write(f"[LAST TX] {self.state['tx']}\n")
                sys.stdout.write("-" * 80 + "\n")
                
                # 受信リスト
                sys.stdout.write(f"{'TIME':<12} | {'CAN ID':<6} | {'CNT':<4} | {'DATA':<23} | {'NAME / DECODED SIGNALS'}\n")
                sys.stdout.write("-" * 120 + "\n")
                
                # ID順にソートして表示
                for arbid in sorted(self.state['rx'].keys()):
                    info = self.state['rx'][arbid]
                    name = info['decoded']['name'] if info['decoded'] else "Unknown"
                    
                    # 信号デコード情報の成形
                    decoded_str = ""
                    if info['decoded']:
                        signals = info['decoded']['signals']
                        parts = []
                        for s_name, s_data in signals.items():
                            parts.append(f"{s_name}:{s_data['value']}")
                        decoded_str = " -> " + ", ".join(parts)
                    
                    sys.stdout.write(
                        f"{info['ts']:<12} | 0x{arbid:03X}  | {info['count']:<4} | {info['data']:<23} | {name}{decoded_str}\n"
                    )
                
                if not self.state['rx']:
                    sys.stdout.write(" Waiting for CAN messages (Try 'candump can0' to check hardware)...\n")
                    
                sys.stdout.write("================================================================================\n")
                sys.stdout.flush()
        except Exception as e:
            # 画面表示自体のエラーは logger で出す
            self.get_logger().error(f"Display error: {e}")

    def decode_can_signals(self, arbid, data):
        """
        CANメッセージをデコードして信号値を抽出
        """
        if arbid not in CAN_SIGNALS:
            return None
        
        msg_def = CAN_SIGNALS[arbid]
        decoded = {'name': msg_def['name'], 'signals': {}}
        
        try:
            for signal_name, signal_info in msg_def['signals'].items():
                signal_type = signal_info['type']
                
                start_byte = signal_info['start_byte']
                length = signal_info['length']
                unit = signal_info.get('unit', '')
                
                # バイト列を取得
                byte_data = data[start_byte:start_byte + length]
                if len(byte_data) < length: continue

                # データ型に応じてデコード
                if signal_type == 'int16':
                    value = struct.unpack('>h', byte_data)[0] # ビッグエンディアンに変更
                elif signal_type == 'uint16':
                    value = struct.unpack('>H', byte_data)[0]
                elif signal_type == 'int32':
                    value = struct.unpack('>i', byte_data)[0]
                elif signal_type == 'uint32':
                    value = struct.unpack('>I', byte_data)[0]
                elif signal_type == 'uint8':
                    value = byte_data[0]
                elif signal_type == 'int8':
                    value = struct.unpack('b', byte_data)[0]
                elif signal_type == 'float32':
                    value = struct.unpack('>f', byte_data)[0]
                else:
                    value = byte_data.hex()
                
                decoded['signals'][signal_name] = {
                    'value': value,
                    'unit': unit
                }
        
        except Exception as e:
            self.get_logger().error(f"Failed to decode signal {signal_name}: {e}")
        
        return decoded

    def status_callback(self):
        """定期的にステータス確認"""
        if self.bus:
            try:
                if self.bus.state.name == 'ACTIVE':
                    self.get_logger().debug("CAN bus is active")
            except:
                pass

    def destroy_node(self):
        """ノードの終了処理"""
        self.running = False
        if self.bus:
            self.bus.shutdown()
        super().destroy_node()


def main(args=None):
    """
    ROS2 ノードのエントリーポイント
    setup.pyのentry_pointsから呼ばれます
    """
    rclpy.init(args=args)
    
    node = MyCanNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()