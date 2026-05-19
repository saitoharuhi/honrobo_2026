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
        result = subprocess.check_output(['hostname', '-I']).decode('utf-8').strip()
        return result.split(' ')[0]
    except:
        return "0.0.0.0"

html_content = """<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<meta charset="utf-8">
<title>Robot Map Control</title>
<style>
/* テキスト選択と長押しメニューを完全に無効化 */
* {
    -webkit-touch-callout: none;
    -webkit-user-select: none;
    -khtml-user-select: none;
    -moz-user-select: none;
    -ms-user-select: none;
    user-select: none;
    outline: none;
    -webkit-tap-highlight-color: transparent;
}

body { 
    font-family: sans-serif; 
    text-align: center; 
    margin: 0; 
    padding: 10px; 
    background-color: #0f172a; 
    color: #f8fafc;
    overflow: hidden;
}

.header h2 { margin: 10px; font-size: 22px; color: #38bdf8; }

.status-bar {
    display: flex;
    justify-content: space-around;
    background: #1e293b;
    padding: 8px;
    border-radius: 10px;
    margin-bottom: 10px;
    font-size: 14px;
    border: 1px solid #334155;
}
.status-val { color: #38bdf8; font-weight: bold; }

/* マップエリア */
.map-container {
    position: relative;
    width: 92vw;
    height: 55vh;
    margin: 0 auto;
    background: #1e293b;
    border: 2px solid #334155;
    border-radius: 15px;
    background-image: 
        linear-gradient(rgba(56, 189, 248, 0.1) 1px, transparent 1px),
        linear-gradient(90deg, rgba(56, 189, 248, 0.1) 1px, transparent 1px);
    background-size: 40px 40px;
}

/* 目的地ボタン */
.loc-btn {
    position: absolute;
    width: 55px;
    height: 55px;
    background: #0ea5e9;
    color: white;
    border: 3px solid #f8fafc;
    border-radius: 50%;
    font-size: 24px;
    font-weight: bold;
    display: flex;
    align-items: center;
    justify-content: center;
    box-shadow: 0 4px 10px rgba(0,0,0,0.5);
    cursor: pointer;
    transform: translate(-50%, -50%);
}
.loc-btn:active {
    background: #f59e0b;
    transform: translate(-50%, -50%) scale(0.9);
}

.stop-btn {
    width: 90%;
    padding: 18px;
    margin-top: 15px;
    background: #ef4444;
    color: white;
    border: none;
    border-radius: 12px;
    font-size: 22px;
    font-weight: bold;
    box-shadow: 0 5px #991b1b;
}
.stop-btn:active {
    box-shadow: 0 2px #991b1b;
    transform: translateY(3px);
}

/* ボタンの配置設定 (マップ上の位置 % ) */
#btn-1 { top: 20%; left: 20%; }
#btn-2 { top: 20%; left: 80%; }
#btn-3 { top: 80%; left: 80%; }
#btn-4 { top: 80%; left: 20%; }
#btn-home { top: 50%; left: 50%; background: #10b981; }

</style>
</head>
<body oncontextmenu="return false;">
    <div class="header">
        <h2>FIELD MAP CONTROL</h2>
    </div>

    <div class="status-bar">
        <div>X: <span id="pos-x">0</span></div>
        <div>Y: <span id="pos-y">0</span></div>
        <div>Z: <span id="pos-z">0</span>°</div>
        <div id="conn-state">🔴</div>
    </div>

    <div class="map-container">
        <div id="btn-1" class="loc-btn" onclick="sendNav(1)">1</div>
        <div id="btn-2" class="loc-btn" onclick="sendNav(2)">2</div>
        <div id="btn-3" class="loc-btn" onclick="sendNav(3)">3</div>
        <div id="btn-4" class="loc-btn" onclick="sendNav(4)">4</div>
        <div id="btn-home" class="loc-btn" onclick="sendNav(0)">H</div>
    </div>

    <button class="stop-btn" onclick="sendStop()">EMERGENCY STOP</button>

<script>
    const wsUrl = "ws://" + window.location.hostname + ":8765";
    let ws;

    function connect() {
        ws = new WebSocket(wsUrl);
        ws.onopen = () => { document.getElementById('conn-state').innerText = '🟢'; };
        ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if(data.type === 'odom') {
                document.getElementById('pos-x').innerText = data.x;
                document.getElementById('pos-y').innerText = data.y;
                document.getElementById('pos-z').innerText = data.yaw;
            }
        };
        ws.onclose = () => {
            document.getElementById('conn-state').innerText = '🔴';
            setTimeout(connect, 2000);
        };
    }

    function sendNav(id) {
        if(ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({action: "navigate_preset", id: id}));
        }
    }

    function sendStop() {
        if(ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({action: "stop"}));
        }
    }
    connect();
</script>
</body>
</html>"""

# --- 目的地座標のプリセット定義 (単位: mm, 度) ---
# ここを書き換えるだけで目的地の位置を調整できます
PRESET_LOCATIONS = {
    0: {"x": 0,  "y": 0,  "yaw": 0},   # Home (Hボタン)
    1: {"x": 10, "y": 30, "yaw": 0},   # 1番
    2: {"x": 60, "y": 30, "yaw": 90},  # 2番
    3: {"x": 60, "y": 70, "yaw": 180}, # 3番
    4: {"x": 10, "y": 70, "yaw": -90}, # 4番
}

class WebNavNode(Node):
    def __init__(self):
        super().__init__('web_nav_node')
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'nav_cmd', 10)
        self.mode_pub = self.create_publisher(Bool, 'auto_mode', 10)
        
        self.current_x, self.current_y, self.current_yaw = 0.0, 0.0, 0.0
        self.target_x, self.target_y, self.target_yaw = None, None, None
        self.is_navigating = False
        
        # 加速度・減速度の個別制限
        self.ACCEL_LIMIT = 200.0  # 発進時の加速の強さ (mm/s^2)
        self.DECEL_LIMIT = 400.0  # 目的地直前のブレーキの強さ (mm/s^2)
        
        self.ANGULAR_ACCEL_LIMIT = 180.0  # 旋回加速度制限 (deg/s^2)
        
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("WebNavNode: Accel/Decel individual control enabled.")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        
    def control_loop(self):
        mode_msg = Bool()
        mode_msg.data = self.is_navigating
        self.mode_pub.publish(mode_msg)
        
        if not self.is_navigating or self.target_x is None:
            self.last_vx, self.last_vy, self.last_vz = 0.0, 0.0, 0.0
            return
            
        dx, dy = self.target_x - self.current_x, self.target_y - self.current_y
        dyaw = math.atan2(math.sin(self.target_yaw - self.current_yaw), math.cos(self.target_yaw - self.current_yaw))
        dist_err = math.hypot(dx, dy)
        
        # 到達判定しきい値を目標座標の 1% に設定 (最低 1mm は確保)
        target_dist_from_origin = math.hypot(self.target_x, self.target_y)
        arrival_threshold = max(0.001, target_dist_from_origin * 0.01)
        
        if dist_err < arrival_threshold and abs(math.degrees(dyaw)) < 3.0:
            self.is_navigating = False
            self.get_logger().info(f"Target Reached! (Threshold: {arrival_threshold:.4f}m)")
            self.send_stop()
            return
            
        # 理想の速度 (P制御)
        local_forward = dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)
        local_left = -dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw)
        
        Kp_pos, Kp_rot, MAX_SPEED = 500.0, 300.0, 500.0
        target_vy = max(-MAX_SPEED, min(MAX_SPEED, local_forward * Kp_pos))
        target_vx = max(-MAX_SPEED, min(MAX_SPEED, local_left * Kp_pos))
        target_vz = max(-MAX_SPEED, min(MAX_SPEED, dyaw * Kp_rot))
        
        # 1ループ 0.02s あたりの最大変化量
        dt = 0.02
        acc_dv = self.ACCEL_LIMIT * dt
        dec_dv = self.DECEL_LIMIT * dt
        
        max_dvz = self.ANGULAR_ACCEL_LIMIT * dt
        
        self.last_vx = self.ramp_velocity(self.last_vx, target_vx, acc_dv, dec_dv)
        self.last_vy = self.ramp_velocity(self.last_vy, target_vy, acc_dv, dec_dv)
        self.last_vz = self.ramp_velocity(self.last_vz, target_vz, max_dvz, max_dvz)
        
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(self.last_vy), float(self.last_vx), float(self.last_vz)
        self.cmd_pub.publish(msg)

    def ramp_velocity(self, current, target, acc_delta, dec_delta):
        """加速時と減速時で異なる制限値を適用して速度を近づける"""
        # 加速（絶対値が増える方向）か、減速（絶対値が減る方向）かを判定
        if abs(target) >= abs(current):
            # 加速
            if target > current:
                return min(target, current + acc_delta)
            else:
                return max(target, current - acc_delta)
        else:
            # 減速
            if target > current:
                return min(target, current + dec_delta)
            else:
                return max(target, current - dec_delta)

    def send_stop(self):
        self.is_navigating = False
        self.last_vx, self.last_vy, self.last_vz = 0.0, 0.0, 0.0
        msg = Twist()
        self.cmd_pub.publish(msg)

    def start_navigation_preset(self, loc_id):
        if loc_id in PRESET_LOCATIONS:
            loc = PRESET_LOCATIONS[loc_id]
            self.target_x = loc['x'] / 1000.0
            self.target_y = loc['y'] / 1000.0
            self.target_yaw = math.radians(loc['yaw'])
            self.is_navigating = True

_ros_node = None

async def websocket_handler(websocket, *args, **kwargs):
    global _ros_node
    async def send_odom():
        while True:
            if _ros_node:
                try:
                    await websocket.send(json.dumps({
                        "type": "odom", 
                        "x": int(_ros_node.current_x * 1000), 
                        "y": int(_ros_node.current_y * 1000), 
                        "yaw": int(math.degrees(_ros_node.current_yaw))
                    }))
                except: break
            await asyncio.sleep(0.1)
    asyncio.create_task(send_odom())
    try:
        async for message in websocket:
            cmd = json.loads(message)
            if cmd.get("action") == "navigate_preset":
                if _ros_node: _ros_node.start_navigation_preset(cmd.get("id"))
            elif cmd.get("action") == "stop":
                if _ros_node: _ros_node.send_stop()
    except: pass

async def main_ws():
    async with websockets.serve(websocket_handler, "0.0.0.0", 8765):
        await asyncio.Future()

def start_websocket_server():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(main_ws())

class UIHandler(SimpleHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(html_content.encode('utf-8'))

def start_http_server():
    HTTPServer(("0.0.0.0", 8080), UIHandler).serve_forever()

def main(args=None):
    global _ros_node
    rclpy.init(args=args)
    _ros_node = WebNavNode()
    threading.Thread(target=start_websocket_server, daemon=True).start()
    threading.Thread(target=start_http_server, daemon=True).start()
    local_ip = get_local_ip()
    print(f"\n[Web UI] http://{local_ip}:8080\n")
    try:
        rclpy.spin(_ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        if _ros_node:
            _ros_node.send_stop()
            _ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
