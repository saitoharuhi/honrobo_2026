"""
web_node.py — WebSocket + HTTP サーバーノード

ブラウザからロボットを操作・監視するインターフェースを提供します。
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
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Int32MultiArray
from sensor_msgs.msg import Joy
from http.server import HTTPServer, SimpleHTTPRequestHandler
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception:
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
# Web UI HTML (統合ダッシュボード)
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
    margin: 0; padding: 10px;
    background: #0f172a; color: #f1f5f9;
    height: 100vh;
    overflow: hidden;
    display: flex;
    flex-direction: column;
}
.header {
    display: flex; justify-content: space-between; align-items: center;
    height: 35px; margin-bottom: 8px;
}
.header h2 { margin: 0; font-size: 18px; color: #38bdf8; font-weight: 600; letter-spacing: 0.5px; }

/* フルスクリーンボタン */
.fs-btn {
    padding: 6px 12px; background: #1e293b; border: 1px solid #334155;
    border-radius: 6px; color: #f1f5f9; font-size: 12px; cursor: pointer;
    display: flex; align-items: center; gap: 5px; transition: all 0.15s ease;
}
.fs-btn:active { background: #334155; }

.status-bar {
    display: grid; grid-template-columns: repeat(5, 1fr);
    background: #1e293b; padding: 6px; border-radius: 10px;
    margin-bottom: 8px; font-size: 11px; border: 1px solid #334155;
    text-align: center; gap: 5px; height: 42px;
}
.status-item { display: flex; flex-direction: column; align-items: center; justify-content: center; }
.status-lbl { color: #64748b; font-size: 9px; text-transform: uppercase; margin-bottom: 1px; }
.status-val { color: #f1f5f9; font-size: 13px; font-weight: bold; font-family: monospace; }
.status-val.highlight { color: #38bdf8; }

/* ダッシュボードメインレイアウト */
.dashboard-container {
    flex: 1;
    display: grid;
    grid-template-columns: 1fr 1.3fr 1.1fr; /* 横画面用3列構成 */
    gap: 10px;
    min-height: 0;
}

.card {
    background: #1e293b; border-radius: 12px; border: 1px solid #334155;
    padding: 10px; display: flex; flex-direction: column; align-items: center;
    justify-content: flex-start; min-height: 0; height: 100%;
}
.card-title {
    align-self: flex-start; margin: 0 0 8px 0; font-size: 12px;
    color: #94a3b8; font-weight: 600; border-left: 3px solid #38bdf8; padding-left: 6px;
}

/* 向きビジュアル（ジャイロ） */
.compass-wrapper {
    position: relative; margin: auto;
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
    background: rgba(56, 189, 248, 0.15);
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
    font-weight: bold; font-family: monospace; color: #f8fafc;
    text-shadow: 0 2px 4px rgba(0,0,0,0.8);
    pointer-events: none;
}

/* ジョイスティック生値の2D表示 */
.joy-pad {
    position: relative;
    background: #0f172a; border-radius: 50%; border: 2px solid #334155;
}
.joy-line-h {
    position: absolute; top: 50%; left: 0; width: 100%; height: 1px; background: #334155;
}
.joy-line-v {
    position: absolute; left: 50%; top: 0; height: 100%; width: 1px; background: #334155;
}
.joy-dot {
    position: absolute; background: #818cf8;
    border-radius: 50%; border: 2px solid #ffffff;
    transform: translate(-50%, -50%);
    top: 50%; left: 50%;
    box-shadow: 0 0 10px rgba(129, 140, 248, 0.6);
    transition: all 0.05s ease-out;
}

/* 速度比較ダッシュボード */
.data-comparison {
    width: 100%; display: flex; flex-direction: column; gap: 4px;
}
.data-row {
    display: flex; justify-content: space-between; align-items: center;
    background: #0f172a; padding: 4px 8px; border-radius: 6px;
    border: 1px solid #1e293b;
}
.data-lbl { color: #94a3b8; font-weight: 500; }
.data-val-pair { display: flex; gap: 8px; font-family: monospace; font-weight: bold; }
.val-joy { color: #818cf8; }
.val-cmd { color: #34d399; }

/* マップ・プリセット制御 */
.map-card {
    background: #1e293b; border-radius: 12px; border: 1px solid #334155;
    padding: 10px; display: flex; flex-direction: column; min-height: 0; height: 100%;
}
.map-container {
    position: relative; width: 100%; flex: 1;
    background: #0f172a; border-radius: 10px; border: 1px solid #334155;
    overflow: hidden;
    background-image:
        linear-gradient(rgba(56,189,248,0.03) 1px, transparent 1px),
        linear-gradient(90deg, rgba(56,189,248,0.03) 1px, transparent 1px);
    background-size: 15px 15px;
}
.map-loc-btn {
    position: absolute; width: 30px; height: 30px; background: #0284c7;
    color: white; border: 2px solid #e2e8f0; border-radius: 50%;
    font-size: 12px; font-weight: bold; display: flex;
    align-items: center; justify-content: center;
    box-shadow: 0 3px 5px rgba(0,0,0,0.3); cursor: pointer;
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
    position: absolute; width: 14px; height: 14px; background: #ef4444;
    border: 2px solid #ffffff; border-radius: 50%;
    transform: translate(-50%, -50%);
    box-shadow: 0 0 8px rgba(239, 68, 68, 0.8);
    pointer-events: none;
    transition: all 0.1s ease-out;
}
.robot-pos-arrow {
    position: absolute; top: -6px; left: 50%; transform: translateX(-50%);
    width: 0; height: 0;
    border-left: 3px solid transparent; border-right: 3px solid transparent;
    border-bottom: 5px solid #ef4444;
}

.stop-btn {
    width: 100%; padding: 10px; background: #dc2626;
    color: white; border: none; border-radius: 10px; font-size: 15px;
    font-weight: bold; box-shadow: 0 3px #991b1b; cursor: pointer;
    transition: all 0.05s ease;
}
.stop-btn:active { box-shadow: 0 1px #991b1b; transform: translateY(2px); }

/* 設定カード */
.config-card {
    background: #1e293b; border-radius: 12px; border: 1px solid #334155;
    padding: 10px; width: 100%; height: 100%; display: flex; flex-direction: column; min-height: 0;
}
.tab-header {
    display: flex; gap: 3px; margin-bottom: 10px; border-bottom: 1px solid #334155;
    padding-bottom: 3px;
}
.tab-btn {
    padding: 6px 8px; background: #0f172a; border: 1px solid #334155;
    border-radius: 6px 6px 0 0; color: #94a3b8; cursor: pointer; font-size: 11px;
    font-weight: 500; transition: all 0.15s ease;
}
.tab-btn.active {
    background: #38bdf8; color: #0f172a; border-color: #38bdf8; font-weight: bold;
}
.config-form {
    display: flex; flex-direction: column; gap: 8px; width: 100%; overflow-y: auto; flex: 1;
}
.form-group {
    display: flex; justify-content: space-between; align-items: center;
    background: #0f172a; padding: 8px 12px; border-radius: 8px;
    border: 1px solid #334155;
}
.form-lbl {
    font-size: 11px; color: #94a3b8; font-weight: 500;
}
.form-input {
    background: #1e293b; border: 1px solid #475569; border-radius: 6px;
    color: #f8fafc; padding: 4px 6px; width: 70px; text-align: center;
    font-family: monospace; font-size: 12px;
}
.form-select {
    background: #1e293b; border: 1px solid #475569; border-radius: 6px;
    color: #f8fafc; padding: 4px 6px; width: 95px; text-align: center;
    font-size: 11px; cursor: pointer;
}
/* 手動実行ボタン */
.action-exec-btn {
    width: 100%; padding: 10px; background: #f59e0b; color: #0f172a;
    border: none; border-radius: 10px; font-size: 14px; font-weight: bold;
    cursor: pointer; transition: all 0.15s ease; text-transform: uppercase;
    box-shadow: 0 3px #b45309; display: none;
}
.action-exec-btn:active {
    box-shadow: 0 1px #b45309; transform: translateY(2px);
}
.action-exec-btn.ready {
    display: block;
    animation: pulse 1.5s infinite alternate;
}
@keyframes pulse {
    0% { box-shadow: 0 0 5px #f59e0b, 0 3px #b45309; }
    100% { box-shadow: 0 0 15px #f59e0b, 0 3px #b45309; }
}

/* ポートレート（縦画面）時はスクロール可能なスタック型へ戻す */
@media (max-width: 767px) and (orientation: portrait) {
    body {
        overflow-y: auto; height: auto;
    }
    .dashboard-container {
        display: flex; flex-direction: column; gap: 12px; height: auto;
    }
    .card, .map-card, .config-card {
        height: auto; min-height: 220px;
    }
}
</style>
</head>
<body oncontextmenu="return false;">
    <div class="header">
        <h2>ROBOT INTEGRATED DASHBOARD</h2>
        <button id="fs-btn" class="fs-btn" onclick="toggleFullscreen()">
            <span>⛶</span> Fullscreen
        </button>
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
            <span class="status-lbl">NAV STATE</span>
            <span id="nav-state" class="status-val highlight">IDLE</span>
        </div>
        <div class="status-item">
            <span class="status-lbl">WS CONN</span>
            <span id="cs" class="status-val">&#x1F534;</span>
        </div>
    </div>

    <div class="dashboard-container">
        <!-- 1列目: 情報・コントロール -->
        <div style="display:flex; flex-direction:column; gap:10px; height:100%; min-height:0;">
            <div class="card" style="flex:1;">
                <h3 class="card-title">ORIENTATION (GYRO)</h3>
                <div class="compass-wrapper" style="width:100px; height:100px;">
                    <div class="compass-ring"></div>
                    <div id="compass-dial" class="compass-dial">
                        <div class="robot-arrow" style="width:22px; height:32px;"></div>
                    </div>
                    <div id="compass-deg" class="compass-degree" style="font-size:13px;">0&deg;</div>
                </div>
            </div>
            <div class="card" style="flex:1.2;">
                <h3 class="card-title">CONTROL & VELOCITY</h3>
                <div style="display:flex; width:100%; align-items:center; gap:8px; flex:1; min-height:0;">
                    <div class="joy-pad" style="width:70px; height:70px; flex-shrink:0;">
                        <div class="joy-line-h"></div>
                        <div class="joy-line-v"></div>
                        <div id="joy-dot" class="joy-dot" style="width:8px; height:8px;"></div>
                    </div>
                    <div class="data-comparison" style="flex:1; font-size:10px; gap:4px;">
                        <div class="data-row" style="padding:4px 6px;">
                            <span class="data-lbl">Stick</span>
                            <div class="data-val-pair">
                                <span id="lbl-joy-lx" class="val-joy">0.00</span>
                                <span id="lbl-joy-ly" class="val-joy">0.00</span>
                            </div>
                        </div>
                        <div class="data-row" style="padding:4px 6px;">
                            <span class="data-lbl">Goal (X, Y)</span>
                            <div class="data-val-pair">
                                <span id="lbl-cmd-vx" class="val-cmd" style="font-size:10px;">0</span>
                                <span id="lbl-cmd-vy" class="val-cmd" style="font-size:10px;">0</span>
                            </div>
                        </div>
                        <div class="data-row" style="padding:4px 6px;">
                            <span class="data-lbl">Goal (Z)</span>
                            <div class="data-val-pair">
                                <span id="lbl-cmd-vz" class="val-cmd" style="font-size:10px;">0</span>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <!-- 2列目: マップ & 操作ボタン -->
        <div style="display:flex; flex-direction:column; gap:10px; height:100%; min-height:0;">
            <div class="map-card" style="flex:1;">
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
            <div style="display:flex; gap:10px; height:38px; flex-shrink:0;">
                <button id="btn-exec-action" class="action-exec-btn" onclick="execAction()" style="flex:1;">EXECUTE ACTION</button>
                <button class="stop-btn" onclick="stp()" style="flex:1;">STOP</button>
            </div>
        </div>

        <!-- 3列目: 設定カード -->
        <div style="display:flex; flex-direction:column; gap:10px; height:100%; min-height:0;">
            <div class="config-card">
                <h3 class="card-title">PRESET CONFIGURATION</h3>
                <div class="tab-header">
                    <button class="tab-btn active" onclick="selectTab(1)">Preset 1</button>
                    <button class="tab-btn" onclick="selectTab(2)">Preset 2</button>
                    <button class="tab-btn" onclick="selectTab(3)">Preset 3</button>
                    <button class="tab-btn" onclick="selectTab(4)">Preset 4</button>
                </div>
                <div class="config-form">
                    <div class="form-group">
                        <span class="form-lbl">動作番号 (Action ID)</span>
                        <input type="number" id="cfg-action-id" class="form-input" min="1" max="99" value="1" onchange="saveConfig()">
                    </div>
                    <div class="form-group">
                        <span class="form-lbl">旋回角度 (Turn Angle)</span>
                        <div style="display:flex; align-items:center; gap:3px;">
                            <input type="number" id="cfg-turn-yaw" class="form-input" min="-180" max="180" value="0" onchange="saveConfig()">
                            <span style="color:#94a3b8; font-size:11px;">&deg;</span>
                        </div>
                    </div>
                    <div class="form-group">
                        <span class="form-lbl">送信タイミング</span>
                        <select id="cfg-action-mode" class="form-select" onchange="saveConfig()">
                            <option value="auto">自動 (Auto)</option>
                            <option value="manual">手動 (Manual)</option>
                        </select>
                    </div>
                </div>
            </div>
        </div>
    </div>

<script>
function toggleFullscreen() {
    if (!document.fullscreenElement) {
        document.documentElement.requestFullscreen().then(() => {
            document.getElementById('fs-btn').innerHTML = '<span>✕</span> Exit';
        }).catch(err => {
            console.error(`Error enabling fullscreen: ${err.message}`);
        });
    } else {
        document.exitFullscreen().then(() => {
            document.getElementById('fs-btn').innerHTML = '<span>⛶</span> Fullscreen';
        });
    }
}
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
            document.getElementById('px').innerText=d.x;
            document.getElementById('py').innerText=d.y;
            document.getElementById('pz').innerText=d.yaw;

            dial.style.transform='rotate('+(d.yaw)+'deg)';
            degLabel.innerText=d.yaw+'\u00B0';

            document.getElementById('lbl-joy-lx').innerText=d.joy_lx.toFixed(2);
            document.getElementById('lbl-joy-ly').innerText=d.joy_ly.toFixed(2);
            const dotX = 50 + (d.joy_lx * 40);
            const dotY = 50 - (d.joy_ly * 40);
            joyDot.style.left = dotX + '%';
            joyDot.style.top = dotY + '%';

            document.getElementById('lbl-cmd-vx').innerText=d.cmd_vx.toFixed(0)+' mm/s';
            document.getElementById('lbl-cmd-vy').innerText=d.cmd_vy.toFixed(0)+' mm/s';
            document.getElementById('lbl-cmd-vz').innerText=d.cmd_vz.toFixed(0)+'\u00B0/s';

            const mapX = 10 + (d.x / 1000.0) * 1.14; 
            const mapY = 80 - (d.y / 1000.0) * 0.85; 
            const clampedX = Math.max(5, Math.min(95, mapX * 100));
            const clampedY = Math.max(5, Math.min(95, mapY * 100));
            marker.style.left = clampedX + '%';
            marker.style.top = clampedY + '%';
            marker.style.transform = 'translate(-50%, -50%) rotate('+d.yaw+'deg)';
        } else if (d.type === 'nav_status') {
            const stateLbl = document.getElementById('nav-state');
            stateLbl.innerText = d.state.toUpperCase().replace(/_/g, ' ');
            if (d.state === 'moving') stateLbl.style.color = '#38bdf8';
            else if (d.state === 'executing_action') stateLbl.style.color = '#f59e0b';
            else if (d.state === 'returning_to_zero') stateLbl.style.color = '#818cf8';
            else stateLbl.style.color = '#34d399';

            const execBtn = document.getElementById('btn-exec-action');
            if (d.state === 'executing_action' && d.action_mode === 'manual') {
                execBtn.classList.add('ready');
            } else {
                execBtn.classList.remove('ready');
            }
        } else if (d.type === 'presets_sync') {
            presetsConfig = d.presets;
            loadConfigToForm();
        }
    };
    w.onclose=()=>{document.getElementById('cs').innerText='\\u1F534';setTimeout(conn,2000);};
}
function nav(id){if(w&&w.readyState===1)w.send(JSON.stringify({action:"navigate_preset",id:id}));}
function stp(){if(w&&w.readyState===1)w.send(JSON.stringify({action:"stop"}));}
function execAction(){if(w&&w.readyState===1)w.send(JSON.stringify({action:"execute_action"}));}

let currentTab = 1;
let presetsConfig = {
    1: { action_id: 1, turn_yaw: 0.0, action_mode: "auto" },
    2: { action_id: 1, turn_yaw: 0.0, action_mode: "auto" },
    3: { action_id: 1, turn_yaw: 0.0, action_mode: "auto" },
    4: { action_id: 1, turn_yaw: 0.0, action_mode: "auto" }
};

function selectTab(id) {
    document.querySelectorAll('.tab-btn').forEach((btn, idx) => {
        if (idx === id - 1) btn.classList.add('active');
        else btn.classList.remove('active');
    });
    currentTab = id;
    loadConfigToForm();
}

function loadConfigToForm() {
    const cfg = presetsConfig[currentTab];
    if (cfg) {
        document.getElementById('cfg-action-id').value = cfg.action_id;
        document.getElementById('cfg-turn-yaw').value = cfg.turn_yaw;
        document.getElementById('cfg-action-mode').value = cfg.action_mode;
    }
}

function saveConfig() {
    presetsConfig[currentTab] = {
        action_id: parseInt(document.getElementById('cfg-action-id').value) || 1,
        turn_yaw: parseFloat(document.getElementById('cfg-turn-yaw').value) || 0.0,
        action_mode: document.getElementById('cfg-action-mode').value
    };
    if (w && w.readyState === 1) {
        w.send(JSON.stringify({
            action: "update_presets",
            presets: presetsConfig
        }));
    }
}

conn();
</script>
</body>
</html>"""


# ============================================================
# 目的地プリセット (ミリメートル、度単位で定義)
# ============================================================
PRESET_LOCATIONS = {
    0: {"x": 0.0, "y": 0.0, "yaw": 0.0, "action_id": 1, "turn_yaw": 0.0, "action_mode": "auto"},
    1: {"x": -1400.0, "y": -2400.0, "yaw": 0.0, "action_id": 5, "turn_yaw": 90.0, "action_mode": "auto"},  # 机1付近 (赤デフォルト)
    2: {"x": -1225.0, "y": -4700.0, "yaw": 0.0, "action_id": 8, "turn_yaw": 180.0, "action_mode": "auto"},  # 旗付近 (赤デフォルト)
    3: {"x": 0.0, "y": -3000.0, "yaw": 0.0, "action_id": 3, "turn_yaw": 0.0, "action_mode": "auto"},
    4: {"x": 0.0, "y": -6000.0, "yaw": 0.0, "action_id": 4, "turn_yaw": 0.0, "action_mode": "auto"},
}

# ステートマシンの状態定義
STATE_IDLE = 'STATE_IDLE'
STATE_NAV_TO_GOAL = 'STATE_NAV_TO_GOAL'
STATE_ACTION = 'STATE_ACTION'
STATE_RETURN_TO_ZERO = 'STATE_RETURN_TO_ZERO'


class WebNavNode(Node):
    def __init__(self):
        super().__init__('web_node')
        
        # サブスクライバー
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)
        self.create_subscription(Joy, 'ps4_joy', self._joy_cb, 10)
        self.create_subscription(Int32MultiArray, 'can_tx', self._can_tx_cb, 10)
        self.create_subscription(Bool, 'auto_mode', self._auto_mode_cb, 10)
        self.create_subscription(OccupancyGrid, 'map', self._map_cb, 10)

        # パブリッシャー
        self.mode_pub = self.create_publisher(Bool, 'auto_mode', 10)
        self.can_pub = self.create_publisher(Int32MultiArray, 'can_tx', 10)
        self.cmd_pub = self.create_publisher(Twist, 'nav_cmd', 10)  # 念のための Twist 停止配信用

        # Nav2 アクションクライアントの初期化
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # ステート管理
        self.state = STATE_IDLE
        self.action_timer = None
        self.goal_handle = None
        self.current_preset_id = None

        self.cur_x, self.cur_y, self.cur_yaw = 0.0, 0.0, 0.0
        self.navigating = False

        # ジョイスティック状態
        self.joy_lx = 0.0
        self.joy_ly = 0.0
        self.joy_rx = 0.0
        self.joy_ry = 0.0

        # CAN目標速度指令(0x510)から逆デコードされた値
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0

        # 動的フットプリント設定用クライアント
        self.global_param_client = self.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')
        self.local_param_client = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')
        self.is_large_footprint = False

    def _map_cb(self, msg):
        """受信したマップデータから赤ゾーンか青ゾーンかを自動判別し、プリセット座標を更新する"""
        w = msg.info.width
        h = msg.info.height
        if w > 10 and h > 100:
            # マップ中央のY行で、左側の壁のピクセル数をスキャン
            y = h // 2
            left_wall_pixels = 0
            for x in range(15):
                idx = y * w + x
                if idx < len(msg.data) and msg.data[idx] > 50:
                    left_wall_pixels += 1
                else:
                    break
            
            # 3ピクセル(150mm)か6ピクセル(300mm)か。閾値は4.5
            is_red = (left_wall_pixels <= 4)
            
            global PRESET_LOCATIONS
            if is_red:
                PRESET_LOCATIONS[1]["x"] = -1400.0
                PRESET_LOCATIONS[1]["y"] = -2400.0
                PRESET_LOCATIONS[2]["x"] = -1225.0
                PRESET_LOCATIONS[2]["y"] = -4700.0
                self.get_logger().info(f"[Zone Detection] RED zone detected (left wall: {left_wall_pixels} px). Coordinates updated.")
            else:
                PRESET_LOCATIONS[1]["x"] = -900.0
                PRESET_LOCATIONS[1]["y"] = -2425.0
                PRESET_LOCATIONS[2]["x"] = -1025.0
                PRESET_LOCATIONS[2]["y"] = -4700.0
                self.get_logger().info(f"[Zone Detection] BLUE zone detected (left wall: {left_wall_pixels} px). Coordinates updated.")

    def _odom_cb(self, msg):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.cur_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y ** 2 + q.z ** 2),
        )

        # 動的フットプリント変更ロジック
        if self.cur_y < -1.2 and not self.is_large_footprint:
            self._set_footprint("large")
        elif self.cur_y >= -0.5 and self.is_large_footprint:
            self._set_footprint("small")

    def _set_footprint(self, size_type):
        """フットプリントを動的に変更する"""
        if size_type == "large":
            footprint_str = "[ [0.525, 0.525], [0.525, -0.525], [-0.525, -0.525], [-0.525, 0.525] ]"
            self.is_large_footprint = True
            self.get_logger().info("Changing footprint to LARGE (1.05m x 1.05m)")
        else:
            footprint_str = "[ [0.475, 0.475], [0.475, -0.475], [-0.475, -0.475], [-0.475, 0.475] ]"
            self.is_large_footprint = False
            self.get_logger().info("Changing footprint to SMALL (0.95m x 0.95m)")

        req = SetParameters.Request()
        param = Parameter()
        param.name = "footprint"
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = footprint_str
        req.parameters = [param]

        if self.global_param_client.service_is_ready():
            self.global_param_client.call_async(req)
        if self.local_param_client.service_is_ready():
            self.local_param_client.call_async(req)

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

    def _auto_mode_cb(self, msg):
        # 手動介入などにより auto_mode が False になった場合、自動走行をキャンセルする
        if not msg.data and self.state != STATE_IDLE:
            self.get_logger().warn("Auto mode disabled externally. Cancelling Nav2 goal.")
            self._stop()

    def _transition(self, new_state):
        self.get_logger().info(f"Transition: {self.state} -> {new_state}")
        self.state = new_state
        
        status_map = {
            STATE_NAV_TO_GOAL: "moving",
            STATE_ACTION: "executing_action",
            STATE_RETURN_TO_ZERO: "returning_to_zero",
            STATE_IDLE: "completed"
        }
        status_str = status_map.get(new_state, "completed")
        self.navigating = (new_state != STATE_IDLE)
        
        loc = PRESET_LOCATIONS.get(self.current_preset_id, {})
        action_mode = loc.get("action_mode", "auto")
        
        # 全WebSocketクライアントへ現在の自律運転ステータスをブロードキャスト
        self.broadcast_to_ws({
            "type": "nav_status", 
            "state": status_str,
            "action_mode": action_mode
        })

    def broadcast_to_ws(self, msg_dict):
        global _ws_loop, active_websockets
        if _ws_loop and active_websockets:
            async def do_broadcast():
                targets = list(active_websockets)
                for ws in targets:
                    try:
                        await ws.send(json.dumps(msg_dict))
                    except Exception:
                        pass
            asyncio.run_coroutine_threadsafe(do_broadcast(), _ws_loop)

    def send_nav2_goal(self, x, y, yaw, done_callback):
        self.get_logger().info(f"Sending Nav2 goal: X={x:.3f}, Y={y:.3f}, Yaw={math.degrees(yaw):.2f}")
        
        # 目標値のアクションゴール作成
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        # クォータニオンへ変換
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cy
        goal_msg.pose.pose.orientation.z = sy
        
        self.nav_to_pose_client.wait_for_server()
        
        self.get_logger().info("Sending goal request...")
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(lambda future: self._goal_response_cb(future, done_callback))

    def _goal_response_cb(self, future, done_callback):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Nav2 goal rejected.")
            self._transition(STATE_IDLE)
            return

        self.get_logger().info("Nav2 goal accepted.")
        self.goal_handle = goal_handle
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(lambda future: self._get_result_cb(future, done_callback))

    def _get_result_cb(self, future, done_callback):
        result = future.result()
        status = result.status
        self.get_logger().info(f"Nav2 goal finished with status: {status}")
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info("Nav2 goal reached successfully.")
            if done_callback:
                done_callback()
        else:
            self.get_logger().error("Nav2 goal failed or was cancelled.")
            self._transition(STATE_IDLE)

    def _stop(self):
        if self.goal_handle is not None and self.navigating:
            self.get_logger().info("Cancelling active Nav2 goal...")
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            
        if self.action_timer is not None:
            self.action_timer.cancel()
            self.action_timer = None

        self._transition(STATE_IDLE)
        self.navigating = False
        
        # /auto_mode を False に
        mode_msg = Bool()
        mode_msg.data = False
        self.mode_pub.publish(mode_msg)
        
        # 速度0をパブリッシュ
        self.cmd_pub.publish(Twist())

    def start_nav_preset(self, loc_id):
        if loc_id not in PRESET_LOCATIONS:
            return
            
        self.current_preset_id = loc_id
        loc = PRESET_LOCATIONS[loc_id]
        
        # /auto_mode を True に
        mode_msg = Bool()
        mode_msg.data = True
        self.mode_pub.publish(mode_msg)
        
        # 自律走行を開始
        self._transition(STATE_NAV_TO_GOAL)
        self.send_nav2_goal(loc['x'] / 1000.0, loc['y'] / 1000.0, math.radians(loc['yaw']), self._on_goal_reached)

    def _on_goal_reached(self):
        loc = PRESET_LOCATIONS.get(self.current_preset_id, {})
        action_mode = loc.get("action_mode", "auto")
        
        if action_mode == "auto":
            self._start_action_sequence()
        else:
            self._transition(STATE_ACTION)
            self.get_logger().info("Arrived at goal. Waiting for manual action execution signal from UI.")

    def _start_action_sequence(self):
        self._transition(STATE_ACTION)
        loc = PRESET_LOCATIONS.get(self.current_preset_id, {})
        turn_yaw_deg = loc.get("turn_yaw", 0.0)
        
        self.get_logger().info(f"Starting action sequence. Turning to target angle: {turn_yaw_deg} deg.")
        
        self.send_nav2_goal(
            self.cur_x, 
            self.cur_y, 
            math.radians(turn_yaw_deg), 
            self._on_turn_completed
        )

    def _on_turn_completed(self):
        loc = PRESET_LOCATIONS.get(self.current_preset_id, {})
        action_id = loc.get("action_id", 1)
        
        self.get_logger().info(f"Turn completed. Sending action ID {action_id} to CAN ID 0x520.")
        
        can_msg = Int32MultiArray()
        can_msg.data = [0x520, action_id, 0, 0, 0, 0, 0, 0]
        self.can_pub.publish(can_msg)
        
        self.get_logger().info("Waiting 5 seconds...")
        if self.action_timer:
            self.action_timer.cancel()
        self.action_timer = self.create_timer(5.0, self._on_wait_timer_completed)

    def _on_wait_timer_completed(self):
        if self.action_timer:
            self.action_timer.cancel()
            self.action_timer = None
            
        loc = PRESET_LOCATIONS.get(self.current_preset_id, {})
        original_yaw_deg = loc.get("yaw", 0.0)
        
        self.get_logger().info(f"5 seconds elapsed. Returning to original angle: {original_yaw_deg} deg.")
        self._transition(STATE_RETURN_TO_ZERO)
        
        self.send_nav2_goal(
            self.cur_x,
            self.cur_y,
            math.radians(original_yaw_deg),
            self._on_return_completed
        )

    def _on_return_completed(self):
        self.get_logger().info("Action sequence completed. Returning to IDLE.")
        self._stop()


_node = None
_ws_loop = None
active_websockets = set()


async def ws_handler(websocket, *args, **kwargs):
    global _node, active_websockets
    active_websockets.add(websocket)
    
    if _node:
        status_map = {
            STATE_NAV_TO_GOAL: "moving",
            STATE_ACTION: "executing_action",
            STATE_RETURN_TO_ZERO: "returning_to_zero",
            STATE_IDLE: "completed"
        }
        loc = PRESET_LOCATIONS.get(_node.current_preset_id, {})
        action_mode = loc.get("action_mode", "auto")
        
        await websocket.send(json.dumps({
            "type": "nav_status",
            "state": status_map.get(_node.state, "completed"),
            "action_mode": action_mode
        }))

        # 現在のプリセット設定をブラウザに送信して UI を同期する
        presets_send = {}
        for pid, data in PRESET_LOCATIONS.items():
            if pid == 0:
                continue
            presets_send[pid] = {
                "action_id": data.get("action_id", 1),
                "turn_yaw": data.get("turn_yaw", 0.0),
                "action_mode": data.get("action_mode", "auto")
            }
        await websocket.send(json.dumps({
            "type": "presets_sync",
            "presets": presets_send
        }))

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
            elif cmd.get("action") == "execute_action" and _node:
                if _node.state == STATE_ACTION:
                    _node.get_logger().info("Manual action execution trigger received from UI.")
                    _node._start_action_sequence()
            elif cmd.get("action") == "update_presets" and _node:
                presets_data = cmd.get("presets", {})
                for pid_str, data in presets_data.items():
                    pid = int(pid_str)
                    if pid in PRESET_LOCATIONS:
                        PRESET_LOCATIONS[pid]["action_id"] = int(data.get("action_id", 1))
                        PRESET_LOCATIONS[pid]["turn_yaw"] = float(data.get("turn_yaw", 0.0))
                        PRESET_LOCATIONS[pid]["action_mode"] = data.get("action_mode", "auto")
                _node.get_logger().info("Presets configuration updated from web UI.")
    except Exception:
        pass
    finally:
        active_websockets.remove(websocket)


async def _ws_main():
    async with websockets.serve(ws_handler, "0.0.0.0", 8765):
        await asyncio.Future()


def _start_ws():
    global _ws_loop
    _ws_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(_ws_loop)
    _ws_loop.run_until_complete(_ws_main())


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
