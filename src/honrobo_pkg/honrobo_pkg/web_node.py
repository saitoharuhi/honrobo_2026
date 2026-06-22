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
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from http.server import HTTPServer, SimpleHTTPRequestHandler


def get_local_ip():
    try:
        result = subprocess.check_output(
            ['hostname', '-I']
        ).decode('utf-8').strip()
        return result.split(' ')[0]
    except Exception:
        return "0.0.0.0"


# ============================================================
# Web UI HTML (マップ型コントローラー)
# ============================================================
HTML_CONTENT = """<!DOCTYPE html>
<html>
<head>
<meta name="viewport"
      content="width=device-width,initial-scale=1.0,maximum-scale=1.0,user-scalable=no">
<meta charset="utf-8">
<title>Robot Map Control</title>
<style>
* {
    -webkit-touch-callout:none; -webkit-user-select:none;
    -moz-user-select:none; -ms-user-select:none; user-select:none;
    outline:none; -webkit-tap-highlight-color:transparent;
}
body {
    font-family:sans-serif; text-align:center; margin:0; padding:10px;
    background:#0f172a; color:#f8fafc; overflow:hidden;
}
.header h2 { margin:10px; font-size:22px; color:#38bdf8; }
.status-bar {
    display:flex; justify-content:space-around; background:#1e293b;
    padding:8px; border-radius:10px; margin-bottom:10px;
    font-size:14px; border:1px solid #334155;
}
.status-val { color:#38bdf8; font-weight:bold; }
.map-container {
    position:relative; width:92vw; height:55vh; margin:0 auto;
    background:#1e293b; border:2px solid #334155; border-radius:15px;
    background-image:
        linear-gradient(rgba(56,189,248,0.1) 1px, transparent 1px),
        linear-gradient(90deg, rgba(56,189,248,0.1) 1px, transparent 1px);
    background-size:40px 40px;
}
.loc-btn {
    position:absolute; width:55px; height:55px; background:#0ea5e9;
    color:white; border:3px solid #f8fafc; border-radius:50%;
    font-size:24px; font-weight:bold; display:flex;
    align-items:center; justify-content:center;
    box-shadow:0 4px 10px rgba(0,0,0,0.5); cursor:pointer;
    transform:translate(-50%,-50%);
}
.loc-btn:active {
    background:#f59e0b; transform:translate(-50%,-50%) scale(0.9);
}
.stop-btn {
    width:90%; padding:18px; margin-top:15px; background:#ef4444;
    color:white; border:none; border-radius:12px; font-size:22px;
    font-weight:bold; box-shadow:0 5px #991b1b;
}
.stop-btn:active { box-shadow:0 2px #991b1b; transform:translateY(3px); }

#btn-1 { top:20%; left:20%; }
#btn-2 { top:20%; left:80%; }
#btn-3 { top:80%; left:80%; }
#btn-4 { top:80%; left:20%; }
#btn-home { top:50%; left:50%; background:#10b981; }
</style>
</head>
<body oncontextmenu="return false;">
    <div class="header"><h2>FIELD MAP CONTROL</h2></div>
    <div class="status-bar">
        <div>X: <span id="px" class="status-val">0</span></div>
        <div>Y: <span id="py" class="status-val">0</span></div>
        <div>Z: <span id="pz" class="status-val">0</span>&deg;</div>
        <div id="cs">&#x1F534;</div>
    </div>
    <div class="map-container">
        <div id="btn-1" class="loc-btn" onclick="nav(1)">1</div>
        <div id="btn-2" class="loc-btn" onclick="nav(2)">2</div>
        <div id="btn-3" class="loc-btn" onclick="nav(3)">3</div>
        <div id="btn-4" class="loc-btn" onclick="nav(4)">4</div>
        <div id="btn-home" class="loc-btn" onclick="nav(0)">H</div>
    </div>
    <button class="stop-btn" onclick="stp()">EMERGENCY STOP</button>
<script>
const u="ws://"+location.hostname+":8765";let w;
function conn(){
    w=new WebSocket(u);
    w.onopen=()=>{document.getElementById('cs').innerText='\\u1F7E2';};
    w.onmessage=(e)=>{
        const d=JSON.parse(e.data);
        if(d.type==='odom'){
            document.getElementById('px').innerText=d.x;
            document.getElementById('py').innerText=d.y;
            document.getElementById('pz').innerText=d.yaw;
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
        self.cmd_pub = self.create_publisher(Twist, 'nav_cmd', 10)
        self.mode_pub = self.create_publisher(Bool, 'auto_mode', 10)

        self.cur_x, self.cur_y, self.cur_yaw = 0.0, 0.0, 0.0
        self.tgt_x, self.tgt_y, self.tgt_yaw = None, None, None
        self.navigating = False

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

        # P制御
        fwd = dx * math.cos(self.cur_yaw) + dy * math.sin(self.cur_yaw)
        lat = -dx * math.sin(self.cur_yaw) + dy * math.cos(self.cur_yaw)
        Kp, Kr, MAX = 500.0, 300.0, 500.0
        t_vy = max(-MAX, min(MAX, fwd * Kp))
        t_vx = max(-MAX, min(MAX, lat * Kp))
        t_vz = max(-MAX, min(MAX, dyaw * Kr))

        dt = 0.02
        a, d = self.ACCEL * dt, self.DECEL * dt
        az = self.ANG_ACCEL * dt

        self.last_vx = self._ramp(self.last_vx, t_vx, a, d)
        self.last_vy = self._ramp(self.last_vy, t_vy, a, d)
        self.last_vz = self._ramp(self.last_vz, t_vz, az, az)

        msg = Twist()
        msg.linear.x = float(self.last_vy)
        msg.linear.y = float(self.last_vx)
        msg.angular.z = float(self.last_vz)
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

    async def send_odom():
        while True:
            if _node:
                try:
                    await websocket.send(json.dumps({
                        "type": "odom",
                        "x": int(_node.cur_x * 1000),
                        "y": int(_node.cur_y * 1000),
                        "yaw": int(math.degrees(_node.cur_yaw)),
                    }))
                except Exception:
                    break
            await asyncio.sleep(0.1)

    asyncio.create_task(send_odom())
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
