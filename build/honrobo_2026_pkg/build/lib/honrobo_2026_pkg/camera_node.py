import rclpy
from rclpy.node import Node
import subprocess
import threading
import signal
import os

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # パラメータの定義 (デフォルトは /dev/video2)
        self.declare_parameter('device', '/dev/video2')
        self.declare_parameter('bitrate', 1000)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.bitrate = self.get_parameter('bitrate').get_parameter_value().integer_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        
        self.process = None
        self.get_logger().info(f"Starting Camera Stream: {self.device} ({self.width}x{self.height} @ {self.bitrate}kbps)")
        
        # GStreamerコマンドの組み立て
        # READMEのコマンドをベースに作成
        self.cmd = [
            'gst-launch-1.0',
            'v4l2src', f'device={self.device}', '!',
            f'video/x-raw,width={self.width},height={self.height},framerate=30/1', '!',
            'videoconvert', '!',
            'vaapih264enc', f'bitrate={self.bitrate}', 'rate-control=cbr', '!',
            'h264parse', '!',
            'rtspclientsink', 'location=rtsp://localhost:8554/mystream', 'latency=0'
        ]
        
        # コマンド実行スレッド
        self.thread = threading.Thread(target=self.run_command, daemon=True)
        self.thread.start()

    def run_command(self):
        try:
            # シェルコマンドとして実行
            self.get_logger().info(f"Executing: {' '.join(self.cmd)}")
            self.process = subprocess.Popen(self.cmd)
            self.process.wait()
        except Exception as e:
            self.get_logger().error(f"Failed to run gstreamer: {e}")

    def stop(self):
        if self.process:
            self.get_logger().info("Stopping GStreamer process...")
            try:
                # 子プロセスだけでなく、プロセスグループ全体にSIGINTを送る
                self.process.send_signal(signal.SIGINT)
                self.process.wait(timeout=2.0)
            except Exception:
                self.process.kill()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # シャットダウン済みでない場合のみ実行
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
