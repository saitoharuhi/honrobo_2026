"""
web_node.py — WebSocket + HTTP サーバーノード

ブラウザからロボットを操作・監視するインターフェースを提供します。

機能:
- HTTP (port 8080): マップUIを配信
- WebSocket (port 8765): リアルタイム双方向通信
- 自動運転: プリセット地点へのナビゲーション(加減速制御付き)

サブスクライブ: /odom (Odometry)
パブリッシュ:   /nav_cmd (Twist), /auto_mode (Bool)
"""

import rclpy
import asyncio
import websockets
import threading
import json
import math
import subprocess
import socket
import struct
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32MultiArray
from sensor_msgs.msg import Joy
from http.server import HTTPServer, SimpleHTTPRequestHandler


def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # 実際にルーティング可能なローカルIPを取得（通信は発生しません）
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception:
        # オフラインや特殊環境時のフォールバック
        try:
            result = subprocess.check_output(
                ['hostname', '-I']
            ).decode('utf-8').strip()
            ip = result.split(' ')[0]
        except Exception:
            ip = "127.0.0.1"
    finally:
        s.close()
    return ip


# ============================================================
# Web UI HTML (マップ型コントローラー)
# ============================================================
HTML_CONTENT = """<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1.0,maximum-scale=1.0,user-scalable=no">
<meta charset="utf-8">
<title>Robot Integrated Dashboard</title>
<style>
* {
    box-sizing: border-box;
    -webkit-touch-callout:none; -webkit-user-select:none;
    -moz-user-select:none; -ms-user-select:none; user-select:none;
    outline:none; -webkit-tap-highlight-color:transparent;
}
body {
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    margin: 0; padding: 15px;
    background: #0f172a; color: #f1f5f9;
    overflow-x: hidden;
}
.header {
    display: flex; justify-content: space-between; align-items: center;
    margin-bottom: 12px;
}
.header h2 { margin: 0; font-size: 20px; color: #38bdf8; font-weight: 600; letter-spacing: 0.5px; }

.status-bar {
    display: grid; grid-template-columns: repeat(4, 1fr);
    background: #1e293b; padding: 10px; border-radius: 12px;
    margin-bottom: 15px; font-size: 13px; border: 1px solid #334155;
    text-align: center; gap: 5px;
}
.status-item { display: flex; flex-direction: column; align-items: center; }
.status-lbl { color: #64748b; font-size: 10px; text-transform: uppercase; margin-bottom: 2px; }
.status-val { color: #f1f5f9; font-size: 15px; font-weight: bold; font-family: monospace; }
.status-val.highlight { color: #38bdf8; }

.main-grid {
    display: grid; grid-template-columns: 1fr 1fr; gap: 15px;
    margin-bottom: 15px;
}
@media (max-width: 600px) {
    .main-grid { grid-template-columns: 1fr; }
}

.card {
    background: #1e293b; border-radius: 16px; border: 1px solid #334155;
    padding: 15px; display: flex; flex-direction: column; align-items: center;
    justify-content: flex-start; min-height: 250px;
}
.card-title {
    align-self: flex-start; margin: 0 0 12px 0; font-size: 13px;
    color: #94a3b8; font-weight: 600; border-left: 3px solid #38bdf8; padding-left: 8px;
}

/* 向きビジュアル（ジャイロ） */
.compass-wrapper {
    position: relative; width: 140px; height: 140px; margin: 10px 0;
}
.compass-ring {
    position: absolute; width: 100%; height: 100%;
    border: 2px dashed #475569; border-radius: 50%;
}
.compass-dial {
    position: absolute; width: 100%; height: 100%;
    transition: transform 0.1s ease-out;
}
.robot-arrow {
    position: absolute; top: 50%; left: 50%;
    width: 32px; height: 48px; background: rgba(56, 189, 248, 0.15);
    border: 3px solid #38bdf8; border-radius: 6px;
    transform: translate(-50%, -50%);
    box-shadow: 0 0 15px rgba(56, 189, 248, 0.4);
}
.robot-arrow::after {
    content: ''; position: absolute; top: -10px; left: 50%;
    transform: translateX(-50%);
    width: 0; height: 0;
    border-left: 8px solid transparent; border-right: 8px solid transparent;
    border-bottom: 10px solid #38bdf8;
}
.compass-degree {
    position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%);
    font-size: 16px; font-weight: bold; font-family: monospace; color: #f8fafc;
    text-shadow: 0 2px 4px rgba(0,0,0,0.8);
    pointer-events: none;
}

/* ジョイスティック生値の2D表示 */
.joy-pad {
    position: relative; width: 100px; height: 100px;
    background: #0f172a; border-radius: 50%; border: 2px solid #334155;
    margin-bottom: 10px;
}
.joy-line-h {
    position: absolute; top: 50%; left: 0; width: 100%; height: 1px; background: #334155;
}
.joy-line-v {
    position: absolute; left: 50%; top: 0; height: 100%; width: 1px; background: #334155;
}
.joy-dot {
    position: absolute; width: 12px; height: 12px; background: #818cf8;
    border-radius: 50%; border: 2px solid #ffffff;
    transform: translate(-50%, -50%);
    top: 50%; left: 50%;
    box-shadow: 0 0 10px rgba(129, 140, 248, 0.6);
    transition: all 0.05s ease-out;
}

/* 速度比較ダッシュボード */
.data-comparison {
    width: 100%; display: flex; flex-direction: column; gap: 6px;
    font-size: 12px;
}
.data-row {
    display: flex; justify-content: space-between; align-items: center;
    background: #0f172a; padding: 6px 10px; border-radius: 8px;
    border: 1px solid #1e293b;
}
.data-lbl { color: #94a3b8; font-weight: 500; }
.data-val-pair { display: flex; gap: 15px; font-family: monospace; font-weight: bold; }
.val-joy { color: #818cf8; }
.val-cmd { color: #34d399; }

/* マップ・プリセット制御 */
.map-card {
    background: #1e293b; border-radius: 16px; border: 1px solid #334155;
    padding: 15px; width: 100%; margin-bottom: 15px;
}
.map-container {
    position: relative; width: 100%; height: 180px;
    background: #0f172a; border-radius: 12px; border: 1px solid #334155;
    overflow: hidden;
    background-image:
        linear-gradient(rgba(56,189,248,0.03) 1px, transparent 1px),
        linear-gradient(90deg, rgba(56,189,248,0.03) 1px, transparent 1px);
    background-size: 20px 20px;
}
.map-loc-btn {
    position: absolute; width: 34px; height: 34px; background: #0284c7;
    color: white; border: 2px solid #e2e8f0; border-radius: 50%;
    font-size: 14px; font-weight: bold; display: flex;
    align-items: center; justify-content: center;
    box-shadow: 0 4px 6px rgba(0,0,0,0.3); cursor: pointer;
    transform: translate(-50%, -50%);
    transition: all 0.1s ease;
}
.map-loc-btn:active { background: #d97706; transform: translate(-50%, -50%) scale(0.9); }
#btn-1 { top: 20%; left: 20%; }
#btn-2 { top: 20%; left: 80%; }
#btn-3 { top: 80%; left: 80%; }
#btn-4 { top: 80%; left: 20%; }
#btn-home { top: 50%; left: 50%; background: #059669; }

.robot-pos-marker {
    position: absolute; width: 16px; height: 16px; background: #ef4444;
    border: 2px solid #ffffff; border-radius: 50%;
    transform: translate(-50%, -50%);
    box-shadow: 0 0 10px rgba(239, 68, 68, 0.8);
    pointer-events: none;
    transition: all 0.1s ease-out;
}
.robot-pos-arrow {
    position: absolute; top: -8px; left: 50%; transform: translateX(-50%);
    width: 0; height: 0;
    border-left: 4px solid transparent; border-right: 4px solid transparent;
    border-bottom: 6px solid #ef4444;
}

.stop-btn {
    width: 100%; padding: 15px; background: #dc2626;
    color: white; border: none; border-radius: 12px; font-size: 18px;
    font-weight: bold; box-shadow: 0 4px #991b1b; cursor: pointer;
    transition: all 0.05s ease;
}
.stop-btn:active { box-shadow: 0 1px #991b1b; transform: translateY(3px); }
</style>
</head>
<body oncontextmenu="return false;">
    <div class="header">
        <h2>ROBOT INTEGRATED DASHBOARD</h2>
    </div>

    <div class="status-bar">
        <div class="status-item">
            <span class="status-lbl">POS X</span>
            <span id="px" class="status-val">0</span>
        </div>
        <div class="status-item">
            <span class="status-lbl">POS Y</span>
            <span id="py" class="status-val">0</span>
        </div>
        <div class="status-item">
            <span class="status-lbl">HEADING</span>
            <span id="pz" class="status-val highlight">0&deg;</span>
        </div>
        <div class="status-item">
            <span class="status-lbl">WS CONN</span>
            <span id="cs" class="status-val">&#x1F534;</span>
        </div>
    </div>

    <div class="main-grid">
        <!-- ジャイロ向きの可視化 -->
        <div class="card">
            <h3 class="card-title">ROBOT ORIENTATION (GYRO)</h3>
            <div class="compass-wrapper">
                <div class="compass-ring"></div>
                <div id="compass-dial" class="compass-dial">
                    <div class="robot-arrow"></div>
                </div>
                <div id="compass-deg" class="compass-degree">0&deg;</div>
            </div>
        </div>

        <!-- スティック値と目標速度のデータ -->
        <div class="card">
            <h3 class="card-title">CONTROL DATA & VELOCITY</h3>
            <div class="joy-pad">
                <div class="joy-line-h"></div>
                <div class="joy-line-v"></div>
                <div id="joy-dot" class="joy-dot"></div>
            </div>
            <div class="data-comparison">
                <div class="data-row">
                    <span class="data-lbl">Stick (LX, LY)</span>
                    <div class="data-val-pair">
                        <span id="lbl-joy-lx" class="val-joy">0.00</span>
                        <span id="lbl-joy-ly" class="val-joy">0.00</span>
                    </div>
                </div>
                <div class="data-row">
                    <span class="data-lbl">Gyro-Comp Goal (VX, VY)</span>
                    <div class="data-val-pair">
                        <span id="lbl-cmd-vx" class="val-cmd">0 mm/s</span>
                        <span id="lbl-cmd-vy" class="val-cmd">0 mm/s</span>
                    </div>
                </div>
                <div class="data-row">
                    <span class="data-lbl">Goal Angular (VZ)</span>
                    <div class="data-val-pair">
                        <span id="lbl-cmd-vz" class="val-cmd">0&deg;/s</span>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <div class="map-card">
        <h3 class="card-title">PRESET DESTINATIONS</h3>
        <div class="map-container">
            <div id="btn-1" class="map-loc-btn" onclick="nav(1)">1</div>
            <div id="btn-2" class="map-loc-btn" onclick="nav(2)">2</div>
            <div id="btn-3" class="map-loc-btn" onclick="nav(3)">3</div>
            <div id="btn-4" class="map-loc-btn" onclick="nav(4)">4</div>
            <div id="btn-home" class="map-loc-btn" onclick="nav(0)">H</div>
            <div id="robot-marker" class="robot-pos-marker">
                <div class="robot-pos-arrow"></div>
            </div>
        </div>
    </div>

    <button class="stop-btn" onclick="stp()">EMERGENCY STOP</button>

<script>
const u="ws://"+location.hostname+":8765";let w;
const dial=document.getElementById('compass-dial');
const degLabel=document.getElementById('compass-deg');
const joyDot=document.getElementById('joy-dot');
const marker=document.getElementById('robot-marker');

function conn(){
    w=new WebSocket(u);
    w.onopen=()=>{document.getElementById('cs').innerText='\\u2705';};
    w.onmessage=(e)=>{
        const d=JSON.parse(e.data);
        if(d.type==='status'){
            // 座標
            document.getElementById('px').innerText=d.x;
            document.getElementById('py').innerText=d.y;
            document.getElementById('pz').innerText=d.yaw;

            // 向き
            dial.style.transform='rotate('+(d.yaw)+'deg)';
            degLabel.innerText=d.yaw+'\u00B0';

            // ジョイスティック
            document.getElementById('lbl-joy-lx').innerText=d.joy_lx.toFixed(2);
            document.getElementById('lbl-joy-ly').innerText=d.joy_ly.toFixed(2);
            const dotX = 50 + (d.joy_lx * 40);
            const dotY = 50 - (d.joy_ly * 40);
            joyDot.style.left = dotX + '%';
            joyDot.style.top = dotY + '%';

            // 目標速度
            document.getElementById('lbl-cmd-vx').innerText=d.cmd_vx.toFixed(0)+' mm/s';
            document.getElementById('lbl-cmd-vy').innerText=d.cmd_vy.toFixed(0)+' mm/s';
            document.getElementById('lbl-cmd-vz').innerText=d.cmd_vz.toFixed(0)+'\u00B0/s';

            // マップ上のマーカー位置 (プリセット座標X:0〜60, Y:0〜70)
            const mapX = 10 + (d.x / 1000.0) * 1.14; // スケール調整
            const mapY = 80 - (d.y / 1000.0) * 0.85; 
            const clampedX = Math.max(5, Math.min(95, mapX * 100));
            const clampedY = Math.max(5, Math.min(95, mapY * 100));
            marker.style.left = clampedX + '%';
            marker.style.top = clampedY + '%';
            marker.style.transform = 'translate(-50%, -50%) rotate('+d.yaw+'deg)';
        }
    };
    w.onclose=()=>{document.getElementById('cs').innerText='\\u1F534';setTimeout(conn,2000);};
}
function nav(id){if(w&&w.readyState===1)w.send(JSON.stringify({action:"navigate_preset",id:id}));}
function stp(){if(w&&w.readyState===1)w.send(JSON.stringify({action:"stop"}));}
conn();
</script>
</body>
</html>"""


# ============================================================
# 目的地プリセット (単位: mm, 度)
# ============================================================
PRESET_LOCATIONS = {
    0: {"x": 0,  "y": 0,  "yaw": 0},     # Home
    1: {"x": 10, "y": 30, "yaw": 0},
    2: {"x": 60, "y": 30, "yaw": 90},
    3: {"x": 60, "y": 70, "yaw": 180},
    4: {"x": 10, "y": 70, "yaw": -90},
}


class WebNavNode(Node):
    def __init__(self):
        super().__init__('web_node')
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)
        self.create_subscription(Joy, 'ps4_joy', self._joy_cb, 10)
        self.create_subscription(Int32MultiArray, 'can_tx', self._can_tx_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, 'nav_cmd', 10)
        self.mode_pub = self.create_publisher(Bool, 'auto_mode', 10)

        self.cur_x, self.cur_y, self.cur_yaw = 0.0, 0.0, 0.0
        self.tgt_x, self.tgt_y, self.tgt_yaw = None, None, None
        self.navigating = False

        # ジョイスティック状態
        self.joy_lx = 0.0
        self.joy_ly = 0.0
        self.joy_rx = 0.0
        self.joy_ry = 0.0

        # CAN目標速度指令(0x510)からデコードされた値
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0

        # 加減速パラメータ
        self.ACCEL = 200.0    # mm/s^2
        self.DECEL = 400.0    # mm/s^2
        self.ANG_ACCEL = 180.0  # deg/s^2
        self.last_vx = self.last_vy = self.last_vz = 0.0

        self.create_timer(0.02, self._control_loop)

    def _odom_cb(self, msg):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.cur_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y ** 2 + q.z ** 2),
        )

    def _joy_cb(self, msg):
        """PS4ジョイスティック生値の取得"""
        if len(msg.axes) >= 4:
            self.joy_lx = msg.axes[0]
            self.joy_ly = msg.axes[1]
            self.joy_rx = msg.axes[2]
            self.joy_ry = msg.axes[3]

    def _can_tx_cb(self, msg):
        """can_txトピックから送信中の目標速度(0x510)を逆デコード"""
        if len(msg.data) >= 7:
            can_id = msg.data[0]
            if can_id == 0x510:
                data_bytes = bytes(msg.data[1:7])
                try:
                    vx, vy, vz = struct.unpack('>hhh', data_bytes)
                    self.cmd_vx = vx / 10.0  # VEL_SCALE=10.0で割る (mm/s)
                    self.cmd_vy = vy / 10.0  # (mm/s)
                    self.cmd_vz = vz / 10.0  # (deg/s)
                except Exception:
                    pass

    def _control_loop(self):
        mode_msg = Bool()
        mode_msg.data = self.navigating
        self.mode_pub.publish(mode_msg)

        if not self.navigating or self.tgt_x is None:
            self.last_vx = self.last_vy = self.last_vz = 0.0
            return

        dx = self.tgt_x - self.cur_x
        dy = self.tgt_y - self.cur_y
        dyaw = math.atan2(
            math.sin(self.tgt_yaw - self.cur_yaw),
            math.cos(self.tgt_yaw - self.cur_yaw),
        )
        dist = math.hypot(dx, dy)

        # 到達判定 (目標距離の1%, 最低1mm)
        threshold = max(0.001, math.hypot(self.tgt_x, self.tgt_y) * 0.01)
        if dist < threshold and abs(math.degrees(dyaw)) < 3.0:
            self.navigating = False
            self.get_logger().info(f"Target reached! (thr={threshold:.4f}m)")
            self._stop()
            return

        # P制御 (最大並進速度 2000mm/s, 最大回転速度 90deg/s)
        fwd = dx * math.cos(self.cur_yaw) + dy * math.sin(self.cur_yaw)
        lat = -dx * math.sin(self.cur_yaw) + dy * math.cos(self.cur_yaw)
        Kp, Kr = 500.0, 300.0
        MAX_SPEED_AUTO = 2000.0
        MAX_ANGULAR_AUTO = 90.0
        t_vy = max(-MAX_SPEED_AUTO, min(MAX_SPEED_AUTO, fwd * Kp))
        t_vx = max(-MAX_SPEED_AUTO, min(MAX_SPEED_AUTO, lat * Kp))
        t_vz = max(-MAX_ANGULAR_AUTO, min(MAX_ANGULAR_AUTO, dyaw * Kr))

        dt = 0.02
        a, d = self.ACCEL * dt, self.DECEL * dt
        az = self.ANG_ACCEL * dt

        self.last_vx = self._ramp(self.last_vx, t_vx, a, d)
        self.last_vy = self._ramp(self.last_vy, t_vy, a, d)
        self.last_vz = self._ramp(self.last_vz, t_vz, az, az)

        msg = Twist()
        msg.linear.x = float(self.last_vx)  # X (左右スライド)
        msg.linear.y = float(self.last_vy)  # Y (前後前進)
        msg.angular.z = float(self.last_vz)  # Z (回転)
        self.cmd_pub.publish(msg)

    @staticmethod
    def _ramp(cur, tgt, acc, dec):
        if abs(tgt) >= abs(cur):
            return min(tgt, cur + acc) if tgt > cur else max(tgt, cur - acc)
        else:
            return min(tgt, cur + dec) if tgt > cur else max(tgt, cur - dec)

    def _stop(self):
        self.navigating = False
        self.last_vx = self.last_vy = self.last_vz = 0.0
        self.cmd_pub.publish(Twist())

    def start_nav_preset(self, loc_id):
        if loc_id in PRESET_LOCATIONS:
            loc = PRESET_LOCATIONS[loc_id]
            self.tgt_x = loc['x'] / 1000.0
            self.tgt_y = loc['y'] / 1000.0
            self.tgt_yaw = math.radians(loc['yaw'])
            self.navigating = True


_node = None


async def ws_handler(websocket, *args, **kwargs):
    global _node

    async def send_status():
        while True:
            if _node:
                try:
                    await websocket.send(json.dumps({
                        "type": "status",
                        "x": int(_node.cur_x * 1000),
                        "y": int(_node.cur_y * 1000),
                        "yaw": int(math.degrees(_node.cur_yaw)),
                        "joy_lx": float(_node.joy_lx),
                        "joy_ly": float(_node.joy_ly),
                        "joy_rx": float(_node.joy_rx),
                        "joy_ry": float(_node.joy_ry),
                        "cmd_vx": float(_node.cmd_vx),
                        "cmd_vy": float(_node.cmd_vy),
                        "cmd_vz": float(_node.cmd_vz),
                        "auto_mode": bool(_node.navigating)
                    }))
                except Exception:
                    break
            await asyncio.sleep(0.05)

    asyncio.create_task(send_status())
    try:
        async for message in websocket:
            cmd = json.loads(message)
            if cmd.get("action") == "navigate_preset" and _node:
                _node.start_nav_preset(cmd.get("id"))
            elif cmd.get("action") == "stop" and _node:
                _node._stop()
    except Exception:
        pass


async def _ws_main():
    async with websockets.serve(ws_handler, "0.0.0.0", 8765):
        await asyncio.Future()


def _start_ws():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(_ws_main())


class RobustHTTPServer(HTTPServer):
    allow_reuse_address = True


class _UIHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(HTML_CONTENT.encode('utf-8'))


def _start_http():
    RobustHTTPServer(("0.0.0.0", 8080), _UIHandler).serve_forever()


def main(args=None):
    global _node
    rclpy.init(args=args)
    _node = WebNavNode()
    threading.Thread(target=_start_ws, daemon=True).start()
    threading.Thread(target=_start_http, daemon=True).start()
    ip = get_local_ip()
    print(f"\n[Web UI] http://{ip}:8080\n")
    try:
        rclpy.spin(_node)
    except KeyboardInterrupt:
        pass
    finally:
        if _node:
            _node._stop()
            _node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
