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
import time
import os
import cv2
import numpy as np
import yaml
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Int32MultiArray, String
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
    background: #f8fafc; color: #0f172a;
    height: 100vh;
    overflow: hidden;
    display: flex;
    flex-direction: column;
}
.header {
    display: flex; justify-content: space-between; align-items: center;
    height: 35px; margin-bottom: 8px;
}
.header h2 { margin: 0; font-size: 16px; color: #0f172a; font-weight: 700; letter-spacing: 0.5px; }

.fs-btn {
    padding: 6px 12px; background: #ffffff; border: 1px solid #cbd5e1;
    border-radius: 6px; color: #0f172a; font-size: 11px; cursor: pointer;
    font-weight: 500; display: flex; align-items: center; gap: 5px; transition: all 0.15s ease;
}
.fs-btn:active { background: #cbd5e1; }

.status-bar {
    display: grid; grid-template-columns: repeat(5, 1fr);
    background: #ffffff; padding: 6px; border-radius: 10px;
    margin-bottom: 8px; font-size: 11px; border: 1px solid #cbd5e1;
    text-align: center; gap: 5px; height: 42px;
}
.status-item { display: flex; flex-direction: column; align-items: center; justify-content: center; }
.status-lbl { color: #64748b; font-size: 9px; font-weight: 600; text-transform: uppercase; margin-bottom: 1px; }
.status-val { color: #0f172a; font-size: 12px; font-weight: bold; font-family: monospace; }
.status-val.highlight { color: #2563eb; }

.dashboard-container {
    flex: 1;
    display: grid;
    grid-template-columns: 1fr 1.3fr 1.1fr;
    gap: 10px;
    min-height: 0;
}

.card {
    background: #ffffff; border-radius: 12px; border: 1px solid #cbd5e1;
    padding: 10px; display: flex; flex-direction: column; align-items: center;
    justify-content: flex-start; min-height: 0; height: 100%;
}
.card-title {
    align-self: flex-start; margin: 0 0 8px 0; font-size: 11px;
    color: #475569; font-weight: bold; border-left: 3px solid #0f172a; padding-left: 6px;
    text-transform: uppercase;
}

.compass-wrapper {
    position: relative; margin: auto;
}
.compass-ring {
    position: absolute; width: 100%; height: 100%;
    border: 1px solid #cbd5e1; border-radius: 50%;
    background: #f8fafc;
}
.compass-dial {
    position: absolute; width: 100%; height: 100%;
    transition: transform 0.1s ease-out;
}
.robot-arrow {
    position: absolute; top: 50%; left: 50%;
    background: rgba(15, 23, 42, 0.05);
    border: 2px solid #0f172a; border-radius: 4px;
    transform: translate(-50%, -50%);
}
.robot-arrow::after {
    content: ''; position: absolute; top: -8px; left: 50%;
    transform: translateX(-50%);
    width: 0; height: 0;
    border-left: 6px solid transparent; border-right: 6px solid transparent;
    border-bottom: 8px solid #0f172a;
}
.compass-degree {
    position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%);
    font-weight: bold; font-family: monospace; color: #0f172a;
    pointer-events: none;
    font-size: 12px;
}

.joy-pad {
    position: relative;
    background: #f8fafc; border-radius: 50%; border: 1px solid #cbd5e1;
}
.joy-line-h {
    position: absolute; top: 50%; left: 0; width: 100%; height: 1px; background: #e2e8f0;
}
.joy-line-v {
    position: absolute; left: 50%; top: 0; height: 100%; width: 1px; background: #e2e8f0;
}
.joy-dot {
    position: absolute; background: #0f172a;
    border-radius: 50%; border: 1px solid #ffffff;
    transform: translate(-50%, -50%);
    top: 50%; left: 50%;
    transition: all 0.05s ease-out;
}

.data-comparison {
    width: 100%; display: flex; flex-direction: column; gap: 4px;
}
.data-row {
    display: flex; justify-content: space-between; align-items: center;
    background: #f8fafc; padding: 4px 8px; border-radius: 6px;
    border: 1px solid #cbd5e1;
}
.data-lbl { color: #64748b; font-weight: 600; }
.data-val-pair { display: flex; gap: 8px; font-family: monospace; font-weight: bold; }
.val-joy { color: #475569; }
.val-cmd { color: #2563eb; }

.map-card {
    background: #ffffff; border-radius: 12px; border: 1px solid #cbd5e1;
    padding: 10px; display: flex; flex-direction: column; min-height: 0; height: 100%;
}
.map-container {
    position: relative; width: 100%; flex: 1;
    background: #f8fafc; border-radius: 10px; border: 1px solid #cbd5e1;
    overflow: hidden;
    display: flex; justify-content: center; align-items: center;
}
.map-wrapper {
    position: relative;
    display: block;
    width: 100%;
    height: 100%;
    cursor: crosshair;
}
.map-img {
    display: block;
    width: 100%;
    height: 100%;
    object-fit: fill;
    border-radius: 4px;
}
.map-loc-btn {
    position: absolute; width: 22px; height: 22px; background: #ffffff;
    color: #0f172a; border: 2px solid #0f172a; border-radius: 50%;
    font-size: 10px; font-weight: bold; display: flex;
    align-items: center; justify-content: center;
    box-shadow: 0 1px 3px rgba(0,0,0,0.15); cursor: pointer;
    transform: translate(-50%, -50%);
    transition: all 0.1s ease;
    z-index: 5;
}
.map-loc-btn:active { background: #cbd5e1; transform: translate(-50%, -50%) scale(0.9); }
#btn-home { background: #0f172a; color: #ffffff; border-color: #0f172a; }

.robot-pos-marker {
    position: absolute; width: 14px; height: 14px; background: rgba(30, 41, 59, 0.9);
    border: 2px solid #38bdf8; border-radius: 3px;
    transform: translate(-50%, -50%);
    box-shadow: 0 0 6px rgba(56, 189, 248, 0.7);
    pointer-events: none;
    display: none;
    box-sizing: border-box;
    z-index: 8;
}
.robot-hitbox-circle {
    position: absolute; top: 50%; left: 50%;
    transform: translate(-50%, -50%);
    border: 1.5px dashed rgba(56, 189, 248, 0.7);
    border-radius: 50%;
    pointer-events: none;
}
.robot-pos-arrow {
    position: absolute; top: -6px; left: 50%; transform: translateX(-50%);
    width: 0; height: 0;
    border-left: 4px solid transparent; border-right: 4px solid transparent;
    border-bottom: 6px solid #f59e0b;
}

.target-pos-marker {
    position: absolute;
    transform: translate(-50%, -50%);
    pointer-events: none;
    z-index: 10;
    display: flex;
    align-items: center;
    justify-content: center;
}
.target-hitbox-circle {
    position: absolute; top: 50%; left: 50%;
    transform: translate(-50%, -50%);
    border: 2px dashed #f43f5e;
    border-radius: 50%;
    background: rgba(244, 63, 94, 0.15);
    pointer-events: none;
}
.target-body-box {
    position: absolute; top: 50%; left: 50%;
    transform: translate(-50%, -50%);
    border: 1.5px solid #0284c7;
    background: rgba(2, 132, 199, 0.18);
    border-radius: 3px;
    pointer-events: none;
}
.target-pin {
    position: relative;
    font-size: 16px;
    line-height: 1;
    z-index: 2;
    filter: drop-shadow(0 1px 2px rgba(0,0,0,0.5));
}
.target-badge {
    position: absolute;
    top: -24px;
    left: 50%;
    transform: translateX(-50%);
    background: rgba(15, 23, 42, 0.9);
    color: #38bdf8;
    font-size: 9px;
    font-weight: 600;
    padding: 2px 6px;
    border-radius: 4px;
    white-space: nowrap;
    border: 1px solid rgba(56, 189, 248, 0.4);
    box-shadow: 0 2px 4px rgba(0,0,0,0.2);
    z-index: 3;
}
.target-badge.clamped {
    background: rgba(120, 53, 15, 0.95);
    color: #fbbf24;
    border-color: #f59e0b;
}
.target-badge.safe {
    background: rgba(6, 78, 59, 0.95);
    color: #34d399;
    border-color: #10b981;
}

.stop-btn {
    width: 100%; padding: 10px; background: #ef4444;
    color: white; border: none; border-radius: 10px; font-size: 14px;
    font-weight: bold; cursor: pointer;
    box-shadow: 0 2px #b91c1c;
    transition: all 0.05s ease;
}
.stop-btn:active { box-shadow: 0 0px #b91c1c; transform: translateY(2px); }

.config-card {
    background: #ffffff; border-radius: 12px; border: 1px solid #cbd5e1;
    padding: 10px; width: 100%; height: 100%; display: flex; flex-direction: column; min-height: 0;
}
.tab-header {
    display: flex; gap: 3px; margin-bottom: 10px; border-bottom: 1px solid #cbd5e1;
    padding-bottom: 3px;
}
.tab-btn {
    padding: 6px 8px; background: #f8fafc; border: 1px solid #cbd5e1;
    border-radius: 6px 6px 0 0; color: #64748b; cursor: pointer; font-size: 10px;
    font-weight: 600; transition: all 0.15s ease;
}
.tab-btn.active {
    background: #0f172a; color: #ffffff; border-color: #0f172a; font-weight: bold;
}
.config-form {
    display: flex; flex-direction: column; gap: 8px; width: 100%; overflow-y: auto; flex: 1;
}
.form-group {
    display: flex; justify-content: space-between; align-items: center;
    background: #f8fafc; padding: 6px 10px; border-radius: 8px;
    border: 1px solid #cbd5e1;
}
.form-lbl {
    font-size: 10px; color: #475569; font-weight: 600;
}
.form-input {
    background: #ffffff; border: 1px solid #cbd5e1; border-radius: 6px;
    color: #0f172a; padding: 4px 6px; width: 70px; text-align: center;
    font-family: monospace; font-size: 11px;
}
.form-select {
    background: #ffffff; border: 1px solid #cbd5e1; border-radius: 6px;
    color: #0f172a; padding: 4px 6px; width: 95px; text-align: center;
    font-size: 10px; cursor: pointer;
}
.action-exec-btn {
    width: 100%; padding: 10px; background: #f59e0b; color: white;
    border: none; border-radius: 10px; font-size: 13px; font-weight: bold;
    cursor: pointer; transition: all 0.15s ease;
    box-shadow: 0 2px #d97706; display: none;
}
.action-exec-btn:active {
    box-shadow: 0 0px #d97706; transform: translateY(2px);
}
.action-exec-btn.ready {
    display: block;
}

@media (max-width: 767px) and (orientation: portrait) {
    body {
        overflow: hidden; height: 100vh; padding: 5px;
    }
    .header {
        height: 30px; margin-bottom: 4px;
    }
    .status-bar {
        height: 35px; margin-bottom: 4px; font-size: 10px; padding: 4px; gap: 3px;
    }
    .status-lbl { font-size: 8px; margin-bottom: 0px; }
    .status-val { font-size: 11px; }

    .dashboard-container {
        display: flex; flex-direction: column; gap: 6px; flex: 1; min-height: 0;
        height: calc(100vh - 85px);
    }
    
    .dashboard-container > div:nth-child(1) {
        flex-direction: row !important; height: 80px !important; flex: 0 0 80px !important; gap: 6px !important;
    }
    .dashboard-container > div:nth-child(1) .card {
        padding: 4px !important; height: 100% !important;
    }
    .compass-wrapper { width: 50px !important; height: 50px !important; }
    .compass-degree { font-size: 9px !important; }
    .robot-arrow { width: 12px !important; height: 18px !important; }
    .joy-pad { width: 45px !important; height: 45px !important; }
    .data-comparison { font-size: 8px !important; }
    
    .dashboard-container > div:nth-child(2) {
        flex: 1 !important; height: auto !important; min-height: 0 !important;
    }
    .map-card {
        padding: 6px !important; height: 100% !important;
    }

    .dashboard-container > div:nth-child(3) {
        flex: 0 0 110px !important; height: 110px !important;
    }
    .config-card {
        padding: 4px 6px !important; height: 100% !important;
    }
    .config-form {
        flex-direction: row !important; flex-wrap: wrap !important; gap: 4px !important;
    }
    .form-group {
        flex: 1 1 45% !important; padding: 3px 6px !important;
    }
    .stop-btn {
        padding: 6px !important; font-size: 12px !important;
    }
}
</style>
</head>
<body oncontextmenu="return false;">
    <div id="disconnect-overlay" style="display:none; position:fixed; top:0; left:0; width:100%; height:100%; background:rgba(0,0,0,0.85); z-index:9999; flex-direction:column; justify-content:center; align-items:center; color:white; font-family:sans-serif;">
        <h1 style="color:#ef4444; font-size:24px; margin-bottom:10px;">⚠️ 通信切断 (Wi-Fi ロスト)</h1>
        <p style="font-size:14px; text-align:center; max-width:80%;">ロボットとの接続が切れました。<br>Wi-Fiの自動復旧を待機しています...</p>
        <div style="margin-top:20px; width:40px; height:40px; border:4px solid #ef4444; border-top:4px solid transparent; border-radius:50%; animation: spin 1s linear infinite;"></div>
    </div>
    <style>@keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }</style>
    <div class="header">
        <h2>ROBOT DASHBOARD</h2>
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
            <span id="cs" class="status-val">🔴</span>
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
                        <div class="robot-arrow" style="width:20px; height:28px;"></div>
                    </div>
                    <div id="compass-deg" class="compass-degree">0&deg;</div>
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
                    <div class="data-comparison" style="flex:1; font-size:9px; gap:3px;">
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
                                <span id="lbl-cmd-vx" class="val-cmd">0</span>
                                <span id="lbl-cmd-vy" class="val-cmd">0</span>
                            </div>
                        </div>
                        <div class="data-row" style="padding:4px 6px;">
                            <span class="data-lbl">Goal (Z)</span>
                            <div class="data-val-pair">
                                <span id="lbl-cmd-vz" class="val-cmd">0</span>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <!-- 2列目: マップ & 操作ボタン -->
        <div style="display:flex; flex-direction:column; gap:10px; height:100%; min-height:0;">
            <div class="map-card" style="flex:1;">
                <div style="display:flex; justify-content:space-between; align-items:center; width:100%; margin-bottom:6px;">
                    <h3 class="card-title" style="margin-bottom:0;">FIELD MAP & TARGETS</h3>
                    <span id="target-info-tag" style="font-size:10px; color:#64748b; font-family:monospace; font-weight:600;">クリックで目的地指定 (当たり判定 R0.48m)</span>
                </div>
                <div class="map-container">
                    <div class="map-wrapper" id="map-wrapper">
                        <img id="map-img" class="map-img" src="/map.png">
                        <div id="btn-1" class="map-loc-btn" onclick="selectPresetNav(1)" style="display:none;">1</div>
                        <div id="btn-2" class="map-loc-btn" onclick="selectPresetNav(2)" style="display:none;">2</div>
                        <div id="btn-3" class="map-loc-btn" onclick="selectPresetNav(3)" style="display:none;">3</div>
                        <div id="btn-4" class="map-loc-btn" onclick="selectPresetNav(4)" style="display:none;">4</div>
                        <div id="btn-home" class="map-loc-btn" onclick="selectPresetNav(0)" style="display:none;">H</div>
                        <div id="target-marker" class="target-pos-marker" style="display:none;">
                            <div id="target-hitbox" class="target-hitbox-circle"></div>
                            <div id="target-body" class="target-body-box"></div>
                            <div class="target-pin">🎯</div>
                            <div id="target-badge" class="target-badge">GOAL</div>
                        </div>
                        <div id="robot-marker" class="robot-pos-marker">
                            <div id="robot-hitbox" class="robot-hitbox-circle"></div>
                            <div class="robot-pos-arrow"></div>
                        </div>
                    </div>
                </div>
            </div>
            <div style="display:flex; gap:8px; height:38px; flex-shrink:0;">
                <button id="btn-nav-target" class="action-exec-btn" onclick="navTarget()" style="flex:1.2; background:#2563eb; display:none; box-shadow:0 2px #1d4ed8;">🚀 GO TO TARGET</button>
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
                        <span class="form-lbl">目標 X (Target X)</span>
                        <div style="display:flex; align-items:center; gap:2px;">
                            <input type="number" id="cfg-pos-x" class="form-input" value="0" step="50" onchange="saveConfig()">
                            <span style="color:#64748b; font-size:10px;">mm</span>
                        </div>
                    </div>
                    <div class="form-group">
                        <span class="form-lbl">目標 Y (Target Y)</span>
                        <div style="display:flex; align-items:center; gap:2px;">
                            <input type="number" id="cfg-pos-y" class="form-input" value="0" step="50" onchange="saveConfig()">
                            <span style="color:#64748b; font-size:10px;">mm</span>
                        </div>
                    </div>
                    <div class="form-group">
                        <span class="form-lbl">動作番号 (Action ID)</span>
                        <input type="number" id="cfg-action-id" class="form-input" min="1" max="99" value="1" onchange="saveConfig()">
                    </div>
                    <div class="form-group">
                        <span class="form-lbl">旋回角度 (Turn Angle)</span>
                        <div style="display:flex; align-items:center; gap:2px;">
                            <input type="number" id="cfg-turn-yaw" class="form-input" min="-180" max="180" value="0" onchange="saveConfig()">
                            <span style="color:#64748b; font-size:10px;">&deg;</span>
                        </div>
                    </div>
                    <div class="form-group">
                        <span class="form-lbl">移動方式 (Nav Type)</span>
                        <select id="cfg-nav-type" class="form-select" onchange="saveConfig()">
                            <option value="nav2">Nav2 (障害物回避)</option>
                            <option value="direct">直線移動 (Direct)</option>
                        </select>
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
let lastMsgTime = Date.now();
setInterval(() => {
    if (Date.now() - lastMsgTime > 3000) {
        document.getElementById('disconnect-overlay').style.display = 'flex';
        document.getElementById('cs').innerText='🔴';
    }
}, 1000);
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
const robotHitbox=document.getElementById('robot-hitbox');
const targetMarker=document.getElementById('target-marker');
const targetHitbox=document.getElementById('target-hitbox');
const targetBody=document.getElementById('target-body');
const targetBadge=document.getElementById('target-badge');
const targetInfoTag=document.getElementById('target-info-tag');
const btnNavTarget=document.getElementById('btn-nav-target');
const img=document.getElementById('map-img');
const wrapper=document.getElementById('map-wrapper');
const container=document.querySelector('.map-container');
let currentMapImage = '';
let latestMapInfo = null;

let customTarget = { x: 2400, y: -1100, yaw: 0.0 };
let currentTab = 1;
let presetsConfig = {
    1: { x: 2400.0, y: -1100.0, action_id: 5, turn_yaw: 0.0, action_mode: "auto", nav_type: "nav2" },
    2: { x: 4550.0, y: -1225.0, action_id: 8, turn_yaw: 90.0, action_mode: "auto", nav_type: "nav2" },
    3: { x: 2400.0, y: -1100.0, action_id: 3, turn_yaw: 0.0, action_mode: "auto", nav_type: "nav2" },
    4: { x: 2400.0, y: 1100.0, action_id: 4, turn_yaw: 0.0, action_mode: "auto", nav_type: "nav2" }
};

function resizeMapWrapper() {
    if (!img.naturalWidth || !img.naturalHeight || !container || !wrapper) return;
    const containerWidth = container.clientWidth;
    const containerHeight = container.clientHeight;
    const imgRatio = img.naturalWidth / img.naturalHeight;
    const containerRatio = containerWidth / containerHeight;
    
    let targetWidth, targetHeight;
    if (imgRatio > containerRatio) {
        targetWidth = containerWidth;
        targetHeight = containerWidth / imgRatio;
    } else {
        targetWidth = containerHeight * imgRatio;
        targetHeight = containerHeight;
    }
    wrapper.style.width = Math.floor(targetWidth) + 'px';
    wrapper.style.height = Math.floor(targetHeight) + 'px';
    if (latestMapInfo) {
        updateMarkerScales(latestMapInfo);
    }
}

img.onload = () => {
    resizeMapWrapper();
};
window.addEventListener('resize', resizeMapWrapper);

function updateMarkerScales(map_info) {
    if (!map_info || map_info.width <= 0) return;
    const containerW = wrapper.clientWidth || 300;
    const ppm = containerW / (map_info.width * map_info.resolution);
    
    // 実機寸法: 車体 0.95m x 0.95m, 当たり判定円 直径 0.96m (R=0.48m)
    const bodyPx = Math.max(12, Math.round(0.95 * ppm));
    const hitboxPx = Math.max(14, Math.round(0.96 * ppm));

    marker.style.width = bodyPx + 'px';
    marker.style.height = bodyPx + 'px';
    if (robotHitbox) {
        robotHitbox.style.width = hitboxPx + 'px';
        robotHitbox.style.height = hitboxPx + 'px';
    }

    if (targetBody) {
        targetBody.style.width = bodyPx + 'px';
        targetBody.style.height = bodyPx + 'px';
    }
    if (targetHitbox) {
        targetHitbox.style.width = hitboxPx + 'px';
        targetHitbox.style.height = hitboxPx + 'px';
    }
}

function renderTargetMarker(x_mm, y_mm, yaw_deg, badgeText) {
    if (!latestMapInfo || latestMapInfo.width <= 0) return;
    const bx = x_mm / 1000.0;
    const by = y_mm / 1000.0;
    const bpx = (bx - latestMapInfo.origin_x) / latestMapInfo.resolution;
    const bpy = latestMapInfo.height - ((by - latestMapInfo.origin_y) / latestMapInfo.resolution);
    const bpctX = (bpx / latestMapInfo.width) * 100;
    const bpctY = (bpy / latestMapInfo.height) * 100;

    targetMarker.style.left = Math.max(0, Math.min(100, bpctX)) + '%';
    targetMarker.style.top = Math.max(0, Math.min(100, bpctY)) + '%';
    
    if (targetBody) {
        targetBody.style.transform = 'translate(-50%, -50%) rotate(' + (90 - yaw_deg) + 'deg)';
    }
    if (badgeText && targetBadge) {
        targetBadge.innerText = badgeText;
    }
    targetMarker.style.display = 'flex';
    btnNavTarget.style.display = 'block';
}

function setCustomTarget(x_mm, y_mm, optional_yaw) {
    const yaw = (optional_yaw !== undefined) ? optional_yaw : (parseFloat(document.getElementById('cfg-turn-yaw').value) || 0.0);
    customTarget = { x: x_mm, y: y_mm, yaw: yaw };

    document.getElementById('cfg-pos-x').value = x_mm;
    document.getElementById('cfg-pos-y').value = y_mm;
    if (presetsConfig[currentTab]) {
        presetsConfig[currentTab].x = x_mm;
        presetsConfig[currentTab].y = y_mm;
    }

    renderTargetMarker(x_mm, y_mm, yaw, 'TARGET');
    targetBadge.className = 'target-badge';
    targetInfoTag.innerText = `目標確認中: (${x_mm}, ${y_mm})`;

    if (w && w.readyState === 1) {
        w.send(JSON.stringify({
            action: "check_target",
            x: x_mm,
            y: y_mm
        }));
    }
}

// マップクリックによる直接指定機能を廃止 (UI入力で正確な座標を指定する方式へ変更)

function conn(){
    w=new WebSocket(u);
    w.onopen=()=>{document.getElementById('cs').innerText='🟢';};
    w.onmessage=(e)=>{
        lastMsgTime = Date.now();
        document.getElementById('disconnect-overlay').style.display = 'none';
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

            if (d.map_info && d.map_info.width > 0) {
                latestMapInfo = d.map_info;
                if (currentMapImage !== d.map_info.image) {
                    currentMapImage = d.map_info.image;
                    img.src = '/map.png?t=' + Date.now();
                }
                resizeMapWrapper();
                updateMarkerScales(d.map_info);

                // ロボットマーカーマッピング
                const rx = d.x / 1000.0;
                const ry = d.y / 1000.0;
                const px = (rx - d.map_info.origin_x) / d.map_info.resolution;
                const py = d.map_info.height - ((ry - d.map_info.origin_y) / d.map_info.resolution);
                const pctX = (px / d.map_info.width) * 100;
                const pctY = (py / d.map_info.height) * 100;
                
                marker.style.left = Math.max(0, Math.min(100, pctX)) + '%';
                marker.style.top = Math.max(0, Math.min(100, pctY)) + '%';
                marker.style.transform = 'translate(-50%, -50%) rotate(' + (90 - d.yaw) + 'deg)';
                marker.style.display = 'block';

                // 各プリセットボタンマッピング
                for (let i = 1; i <= 4; i++) {
                    const btn = document.getElementById('btn-' + i);
                    const cfg = presetsConfig[i];
                    if (cfg && cfg.hasOwnProperty('x') && cfg.hasOwnProperty('y')) {
                        const bx = cfg.x / 1000.0;
                        const by = cfg.y / 1000.0;
                        const bpx = (bx - d.map_info.origin_x) / d.map_info.resolution;
                        const bpy = d.map_info.height - ((by - d.map_info.origin_y) / d.map_info.resolution);
                        const bpctX = (bpx / d.map_info.width) * 100;
                        const bpctY = (bpy / d.map_info.height) * 100;
                        
                        btn.style.left = Math.max(0, Math.min(100, bpctX)) + '%';
                        btn.style.top = Math.max(0, Math.min(100, bpctY)) + '%';
                        btn.style.display = 'flex';
                    }
                }

                // Home ボタン (0, 0) マッピング
                const homeBtn = document.getElementById('btn-home');
                const hpx = (0.0 - d.map_info.origin_x) / d.map_info.resolution;
                const hpy = d.map_info.height - ((0.0 - d.map_info.origin_y) / d.map_info.resolution);
                const hpctX = (hpx / d.map_info.width) * 100;
                const hpctY = (hpy / d.map_info.height) * 100;
                homeBtn.style.left = Math.max(0, Math.min(100, hpctX)) + '%';
                homeBtn.style.top = Math.max(0, Math.min(100, hpctY)) + '%';
                homeBtn.style.display = 'flex';
            }
        } else if (d.type === 'target_checked') {
            customTarget.x = d.x;
            customTarget.y = d.y;
            document.getElementById('cfg-pos-x').value = d.x;
            document.getElementById('cfg-pos-y').value = d.y;
            if (presetsConfig[currentTab]) {
                presetsConfig[currentTab].x = d.x;
                presetsConfig[currentTab].y = d.y;
            }
            const yaw = parseFloat(document.getElementById('cfg-turn-yaw').value) || 0.0;
            if (d.clamped) {
                renderTargetMarker(d.x, d.y, yaw, `⚠️ 補正済 (${d.clearance}mm)`);
                targetBadge.className = 'target-badge clamped';
                targetInfoTag.innerText = `⚠️ 机/壁接近のため安全位置へ自動補正 (余裕: ${d.clearance}mm)`;
                targetInfoTag.style.color = '#f59e0b';
            } else {
                renderTargetMarker(d.x, d.y, yaw, `✔️ 安全 (${d.clearance}mm)`);
                targetBadge.className = 'target-badge safe';
                targetInfoTag.innerText = `✔️ 安全位置 (余裕: ${d.clearance}mm)`;
                targetInfoTag.style.color = '#10b981';
            }
        } else if (d.type === 'target_confirmed') {
            customTarget.x = d.x;
            customTarget.y = d.y;
            const yaw = parseFloat(document.getElementById('cfg-turn-yaw').value) || 0.0;
            renderTargetMarker(d.x, d.y, yaw, `🚀 進行中 (${d.clearance}mm)`);
            if (d.clamped) {
                targetBadge.className = 'target-badge clamped';
                targetInfoTag.innerText = `🚀 目的地へ走行中 (当たり判定補正済: 余裕 ${d.clearance}mm)`;
                targetInfoTag.style.color = '#f59e0b';
            } else {
                targetBadge.className = 'target-badge safe';
                targetInfoTag.innerText = `🚀 目的地へ走行中 (余裕: ${d.clearance}mm)`;
                targetInfoTag.style.color = '#2563eb';
            }
        } else if (d.type === 'nav_status') {
            const stateLbl = document.getElementById('nav-state');
            stateLbl.innerText = d.state.toUpperCase().replace(/_/g, ' ');
            if (d.state === 'moving') stateLbl.style.color = '#2563eb';
            else if (d.state === 'executing_action') stateLbl.style.color = '#f59e0b';
            else if (d.state === 'returning_to_zero') stateLbl.style.color = '#475569';
            else stateLbl.style.color = '#10b981';

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
    w.onopen = () => {
        document.getElementById('cs').innerText='🟢';
        document.getElementById('disconnect-overlay').style.display = 'none';
    };
    w.onclose = () => {
        document.getElementById('cs').innerText='🔴';
        document.getElementById('disconnect-overlay').style.display = 'flex';
        setTimeout(conn, 2000);
    };
}

let gpInterval = null;
window.addEventListener("gamepadconnected", (e) => {
    console.log("Gamepad connected!");
    if (!gpInterval) {
        gpInterval = setInterval(() => {
            const gps = navigator.getGamepads();
            const gp = gps[0];
            if (gp && w && w.readyState === 1) {
                let out_axes = [
                    gp.axes[0], 
                    -gp.axes[1], 
                    gp.axes[2], 
                    -gp.axes[3], 
                    gp.buttons[6]?.value || 0.0,
                    gp.buttons[7]?.value || 0.0
                ];
                let out_btns = [
                    gp.buttons[2]?.pressed ? 1 : 0, // Square
                    gp.buttons[0]?.pressed ? 1 : 0, // Cross
                    gp.buttons[1]?.pressed ? 1 : 0, // Circle
                    gp.buttons[3]?.pressed ? 1 : 0, // Triangle
                    gp.buttons[4]?.pressed ? 1 : 0, // L1
                    gp.buttons[5]?.pressed ? 1 : 0, // R1
                    gp.buttons[6]?.pressed ? 1 : 0, // L2
                    gp.buttons[7]?.pressed ? 1 : 0, // R2
                    gp.buttons[8]?.pressed ? 1 : 0, // Share
                    gp.buttons[9]?.pressed ? 1 : 0, // Options
                    gp.buttons[16]?.pressed ? 1 : 0, // PS
                    gp.buttons[10]?.pressed ? 1 : 0, // L3
                    gp.buttons[11]?.pressed ? 1 : 0, // R3
                    gp.buttons[12]?.pressed ? 1 : 0, // UP
                    gp.buttons[13]?.pressed ? 1 : 0, // DOWN
                    gp.buttons[14]?.pressed ? 1 : 0, // LEFT
                    gp.buttons[15]?.pressed ? 1 : 0, // RIGHT
                ];
                w.send(JSON.stringify({
                    action: "gamepad",
                    axes: out_axes,
                    buttons: out_btns
                }));
            }
        }, 50); // 20Hz (0.05s)
    }
});
window.addEventListener("gamepaddisconnected", (e) => {
    if (gpInterval) {
        clearInterval(gpInterval);
        gpInterval = null;
    }
});

function nav(id){if(w&&w.readyState===1)w.send(JSON.stringify({action:"navigate_preset",id:id}));}
function stp(){if(w&&w.readyState===1)w.send(JSON.stringify({action:"stop"}));}
function execAction(){if(w&&w.readyState===1)w.send(JSON.stringify({action:"execute_action"}));}

function selectPresetNav(id) {
    if (id >= 1 && id <= 4) {
        selectTab(id);
    } else if (id === 0) {
        setCustomTarget(0, 0, 0.0);
    }
    nav(id);
}

function navTarget() {
    if (w && w.readyState === 1) {
        const navType = document.getElementById('cfg-nav-type').value || "nav2";
        const turnYaw = parseFloat(document.getElementById('cfg-turn-yaw').value) || 0.0;
        w.send(JSON.stringify({
            action: "navigate_point",
            x: customTarget.x,
            y: customTarget.y,
            yaw: turnYaw,
            nav_type: navType
        }));
    }
}

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
        document.getElementById('cfg-pos-x').value = Math.round(cfg.x || 0);
        document.getElementById('cfg-pos-y').value = Math.round(cfg.y || 0);
        document.getElementById('cfg-action-id').value = cfg.action_id;
        document.getElementById('cfg-turn-yaw').value = cfg.turn_yaw;
        document.getElementById('cfg-nav-type').value = cfg.nav_type || "nav2";
        document.getElementById('cfg-action-mode').value = cfg.action_mode;

        if (cfg.hasOwnProperty('x') && cfg.hasOwnProperty('y')) {
            customTarget = { x: cfg.x, y: cfg.y, yaw: cfg.turn_yaw };
            renderTargetMarker(cfg.x, cfg.y, cfg.turn_yaw, 'P' + currentTab);
            if (w && w.readyState === 1) {
                w.send(JSON.stringify({ action: "check_target", x: cfg.x, y: cfg.y }));
            }
        }
    }
}

function saveConfig() {
    const x = parseFloat(document.getElementById('cfg-pos-x').value) || 0.0;
    const y = parseFloat(document.getElementById('cfg-pos-y').value) || 0.0;
    presetsConfig[currentTab].x = x;
    presetsConfig[currentTab].y = y;
    presetsConfig[currentTab].action_id = parseInt(document.getElementById('cfg-action-id').value) || 1;
    presetsConfig[currentTab].turn_yaw = parseFloat(document.getElementById('cfg-turn-yaw').value) || 0.0;
    presetsConfig[currentTab].nav_type = document.getElementById('cfg-nav-type').value;
    presetsConfig[currentTab].action_mode = document.getElementById('cfg-action-mode').value;
    
    customTarget = { x: x, y: y, yaw: presetsConfig[currentTab].turn_yaw };
    renderTargetMarker(x, y, customTarget.yaw, 'P' + currentTab);

    if (w && w.readyState === 1) {
        w.send(JSON.stringify({
            action: "check_target",
            x: x,
            y: y
        }));
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
    0: {"x": 0.0, "y": 0.0, "yaw": 0.0, "action_id": 1, "turn_yaw": 0.0, "action_mode": "auto", "nav_type": "nav2"},
    1: {"x": 2400.0, "y": -1100.0, "yaw": 0.0, "action_id": 5, "turn_yaw": 0.0, "action_mode": "auto", "nav_type": "nav2"},  # 机1前 (クリアランス 0.50m)
    2: {"x": 4550.0, "y": -1225.0, "yaw": 90.0, "action_id": 8, "turn_yaw": 90.0, "action_mode": "auto", "nav_type": "nav2"},  # 旗前 (クリアランス 0.50m)
    3: {"x": 2400.0, "y": -1100.0, "yaw": 0.0, "action_id": 3, "turn_yaw": 0.0, "action_mode": "auto", "nav_type": "nav2"},  # 赤机前
    4: {"x": 2400.0, "y": 1100.0, "yaw": 0.0, "action_id": 4, "turn_yaw": 0.0, "action_mode": "auto", "nav_type": "nav2"},   # 青机前
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
        self.ip_pub = self.create_publisher(String, 'robot_ip', 10)
        self.ps4_pub = self.create_publisher(Joy, 'ps4_joy', 10)

        # FastNav (C++ 超高速ナビ) 連携用
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.nav_status_sub = self.create_subscription(String, 'nav_status', self._nav_status_cb, 10)
        self.nav_done_cb = None

        # Nav2 アクションクライアントの初期化 (フォールバック互換性維持)
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

        # 直線移動自動運転(Direct Drive)制御用
        self.control_timer = None
        self.direct_start_x = 0.0
        self.direct_start_y = 0.0
        self.direct_target_x = 0.0
        self.direct_target_y = 0.0
        self.direct_target_yaw = 0.0
        self.direct_line_angle = 0.0
        self.direct_total_dist = 0.0
        self.direct_done_cb = None
        self.direct_phase = 'move'  # 'move' または 'turn'

        # マップ情報格納用
        self.map_width = 1.0
        self.map_height = 1.0
        self.map_resolution = 0.05
        self.map_origin_x = -0.5
        self.map_origin_y = -0.5
        self.map_image_name = "map_red.png"
        self.dist_map = None

        # 初期マップの読み込み (トピック受信前の当たり判定・クリアランス計算用)
        self._load_initial_map()

        # IPアドレス定期配信タイマー (1分周期)
        self.ip_address_str = get_local_ip()
        self._publish_ip()  # 起動直後に即座に1回配信
        self.create_timer(60.0, self._publish_ip)

    def _load_initial_map(self):
        """起動時に map_red.yaml / map_red.png をロードして初期の当たり判定用距離マップを構築"""
        try:
            from ament_index_python.packages import get_package_share_directory
            yaml_path = None
            try:
                pkg_share = get_package_share_directory('honrobo_pkg')
                yaml_path = os.path.join(pkg_share, 'map', 'map_red.yaml')
            except Exception:
                pass
            if not yaml_path or not os.path.exists(yaml_path):
                yaml_path = "/home/haru/Documents/honrobo_2026/src/honrobo_pkg/map/map_red.yaml"

            if os.path.exists(yaml_path):
                with open(yaml_path, 'r') as f:
                    meta = yaml.safe_load(f)
                self.map_resolution = float(meta.get('resolution', 0.05))
                origin = meta.get('origin', [-0.5, -0.5, 0.0])
                self.map_origin_x = float(origin[0])
                self.map_origin_y = float(origin[1])

                img_file = meta.get('image', 'map_red.png')
                img_path = os.path.join(os.path.dirname(yaml_path), img_file)
                if os.path.exists(img_path):
                    cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                    if cv_img is not None:
                        h, w = cv_img.shape
                        self.map_width = float(w)
                        self.map_height = float(h)
                        cv_ros = cv2.flip(cv_img, 0)
                        bin_grid = np.where(cv_ros < 50, 0, 255).astype(np.uint8)
                        self.dist_map = cv2.distanceTransform(bin_grid, cv2.DIST_L2, 5) * self.map_resolution
                        self.get_logger().info(f"Loaded initial fallback map: {w}x{h}, res={self.map_resolution:.3f}")
        except Exception as e:
            self.get_logger().warn(f"Failed to load initial map: {e}")

    def adjust_goal_for_hitbox(self, wx, wy):
        """
        目標地点の当たり判定チェック & 自動補正
        機体の当たり判定半径 0.48m (950mm正方形の外接円/衝突半径) に対し、
        20mmの安全マージンを加えた 0.50m (500mm) のクリアランスを確保する。
        もし目標地点の障害物距離が 0.48m 未満（壁や机にめり込んでいる）場合、
        最も近い安全な地点 (クリアランス >= 0.50m) へ自動補正する。
        """
        if self.dist_map is None:
            return wx, wy, 0.50, False

        h, w = self.dist_map.shape
        col = int(round((wx - self.map_origin_x) / self.map_resolution))
        row = int(round((wy - self.map_origin_y) / self.map_resolution))

        if col < 0 or col >= w or row < 0 or row >= h:
            return wx, wy, 0.0, False

        cur_cl = float(self.dist_map[row, col])
        if cur_cl >= 0.48:
            return wx, wy, cur_cl, False

        # 障害物・机に近すぎるため最近傍安全セルを探索 (クリアランス >= 0.50m)
        best_dist_sq = float('inf')
        best_cell = None
        max_search_radius = int(1.5 / self.map_resolution)

        for dr in range(-max_search_radius, max_search_radius + 1):
            nr = row + dr
            if nr < 0 or nr >= h:
                continue
            for dc in range(-max_search_radius, max_search_radius + 1):
                nc = col + dc
                if nc < 0 or nc >= w:
                    continue
                if self.dist_map[nr, nc] >= 0.50:
                    d_sq = dr * dr + dc * dc
                    if d_sq < best_dist_sq:
                        best_dist_sq = d_sq
                        best_cell = (nr, nc)

        if best_cell is not None:
            safe_wx = self.map_origin_x + best_cell[1] * self.map_resolution
            safe_wy = self.map_origin_y + best_cell[0] * self.map_resolution
            new_cl = float(self.dist_map[best_cell[0], best_cell[1]])
            self.get_logger().info(
                f"[Hitbox Adjust] Target ({wx:.3f}, {wy:.3f}, cl={cur_cl:.3f}m) clamped to "
                f"safe point ({safe_wx:.3f}, {safe_wy:.3f}, cl={new_cl:.3f}m)"
            )
            return safe_wx, safe_wy, new_cl, True

        return wx, wy, cur_cl, False

    def broadcast_presets(self):
        """現在の全プリセット情報を WebSocket クライアントへ配信"""
        presets_send = {}
        for pid, data in PRESET_LOCATIONS.items():
            if pid == 0:
                continue
            presets_send[pid] = {
                "x": float(data.get("x", 0.0)),
                "y": float(data.get("y", 0.0)),
                "action_id": data.get("action_id", 1),
                "turn_yaw": data.get("turn_yaw", 0.0),
                "nav_type": data.get("nav_type", "nav2"),
                "action_mode": data.get("action_mode", "auto")
            }
        self.broadcast_to_ws({
            "type": "presets_sync",
            "presets": presets_send
        })

    def _publish_ip(self):
        msg = String()
        msg.data = self.ip_address_str
        self.ip_pub.publish(msg)

    def _map_cb(self, msg):
        """受信したマップデータから赤ゾーン・青ゾーン・テストマップを自動判別し、プリセット座標や配信情報を更新する"""
        w = msg.info.width
        h = msg.info.height
        self.map_width = float(w)
        self.map_height = float(h)
        self.map_resolution = float(msg.info.resolution)
        self.map_origin_x = float(msg.info.origin.position.x)
        self.map_origin_y = float(msg.info.origin.position.y)

        # 障害物距離マップのリアルタイム更新
        try:
            grid = np.array(msg.data, dtype=np.int8).reshape((h, w))
            bin_grid = np.where((grid > 50) | (grid < 0), 0, 255).astype(np.uint8)
            self.dist_map = cv2.distanceTransform(bin_grid, cv2.DIST_L2, 5) * self.map_resolution
        except Exception as e:
            self.get_logger().warn(f"Failed to calculate distance transform in _map_cb: {e}")

        if w == 40 and h == 40:
            self.map_image_name = "map_test.png"
            self.get_logger().info("[Zone Detection] TEST zone detected (40x40). Map image set to map_test.png")
        elif w > 10 and h > 100:
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
                self.map_image_name = "map_red.png"
                p1_x, p1_y, _, _ = self.adjust_goal_for_hitbox(2.4, -1.4)
                p2_x, p2_y, _, _ = self.adjust_goal_for_hitbox(4.7, -1.225)
                p3_x, p3_y, _, _ = self.adjust_goal_for_hitbox(2.4, -1.4)
                p4_x, p4_y, _, _ = self.adjust_goal_for_hitbox(2.4, 1.4)
                PRESET_LOCATIONS[1]["x"] = round(p1_x * 1000.0)
                PRESET_LOCATIONS[1]["y"] = round(p1_y * 1000.0)
                PRESET_LOCATIONS[1]["turn_yaw"] = 0.0
                PRESET_LOCATIONS[2]["x"] = round(p2_x * 1000.0)
                PRESET_LOCATIONS[2]["y"] = round(p2_y * 1000.0)
                PRESET_LOCATIONS[2]["turn_yaw"] = 90.0
                PRESET_LOCATIONS[3]["x"] = round(p3_x * 1000.0)
                PRESET_LOCATIONS[3]["y"] = round(p3_y * 1000.0)
                PRESET_LOCATIONS[3]["turn_yaw"] = 0.0
                PRESET_LOCATIONS[4]["x"] = round(p4_x * 1000.0)
                PRESET_LOCATIONS[4]["y"] = round(p4_y * 1000.0)
                PRESET_LOCATIONS[4]["turn_yaw"] = 0.0
                self.get_logger().info(f"[Zone Detection] RED zone detected (left wall: {left_wall_pixels} px). Hitbox-safe coordinates updated.")
            else:
                self.map_image_name = "map_blue.png"
                p1_x, p1_y, _, _ = self.adjust_goal_for_hitbox(2.425, 0.95)
                p2_x, p2_y, _, _ = self.adjust_goal_for_hitbox(4.7, 1.15)
                p3_x, p3_y, _, _ = self.adjust_goal_for_hitbox(2.4, 0.95)
                p4_x, p4_y, _, _ = self.adjust_goal_for_hitbox(2.4, -1.1)
                PRESET_LOCATIONS[1]["x"] = round(p1_x * 1000.0)
                PRESET_LOCATIONS[1]["y"] = round(p1_y * 1000.0)
                PRESET_LOCATIONS[1]["turn_yaw"] = 0.0
                PRESET_LOCATIONS[2]["x"] = round(p2_x * 1000.0)
                PRESET_LOCATIONS[2]["y"] = round(p2_y * 1000.0)
                PRESET_LOCATIONS[2]["turn_yaw"] = 90.0
                PRESET_LOCATIONS[3]["x"] = round(p3_x * 1000.0)
                PRESET_LOCATIONS[3]["y"] = round(p3_y * 1000.0)
                PRESET_LOCATIONS[3]["turn_yaw"] = 0.0
                PRESET_LOCATIONS[4]["x"] = round(p4_x * 1000.0)
                PRESET_LOCATIONS[4]["y"] = round(p4_y * 1000.0)
                PRESET_LOCATIONS[4]["turn_yaw"] = 0.0
                self.get_logger().info(f"[Zone Detection] BLUE zone detected (left wall: {left_wall_pixels} px). Hitbox-safe coordinates updated.")

            self.broadcast_presets()

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

    def send_goal(self, x, y, yaw, done_callback):
        safe_x, safe_y, cl, clamped = self.adjust_goal_for_hitbox(x, y)
        if clamped:
            self.get_logger().warn(
                f"[Hitbox Safety] Clamped goal ({x:.3f}, {y:.3f}) -> ({safe_x:.3f}, {safe_y:.3f}), cl={cl:.3f}m"
            )
        loc = PRESET_LOCATIONS.get(self.current_preset_id, {})
        nav_type = loc.get("nav_type", "nav2")
        if nav_type == "direct":
            self.start_direct_drive_goal(safe_x, safe_y, yaw, done_callback)
        else:
            self.send_nav2_goal(safe_x, safe_y, yaw, done_callback)

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def start_direct_drive_goal(self, target_x, target_y, target_yaw, done_callback):
        # 自動運転開始の瞬間の位置を記録
        self.direct_start_x = self.cur_x
        self.direct_start_y = self.cur_y
        self.direct_target_x = target_x
        self.direct_target_y = target_y
        self.direct_target_yaw = target_yaw
        self.direct_done_cb = done_callback
        
        dx = target_x - self.direct_start_x
        dy = target_y - self.direct_start_y
        self.direct_total_dist = math.hypot(dx, dy)
        self.direct_line_angle = math.atan2(dy, dx)
        
        self.get_logger().info(
            f"[Direct Drive] Started from X0={self.direct_start_x:.3f}, Y0={self.direct_start_y:.3f} "
            f"-> Goal X={target_x:.3f}, Y={target_y:.3f} (Dist={self.direct_total_dist*1000:.0f}mm, LineAngle={math.degrees(self.direct_line_angle):.1f}deg)"
        )
        
        if self.direct_total_dist > 0.05:
            self.direct_phase = 'move'
        else:
            self.direct_phase = 'turn'
            
        if self.control_timer:
            self.control_timer.cancel()
        self.control_timer = self.create_timer(0.05, self._direct_control_step)

    def _direct_control_step(self):
        if not self.navigating and self.state == STATE_IDLE:
            if self.control_timer:
                self.control_timer.cancel()
                self.control_timer = None
            return

        cmd = Twist()
        if self.direct_phase == 'move':
            # 目標までの残り距離およびスタート地点からの移動距離
            d_remain = math.hypot(self.direct_target_x - self.cur_x, self.direct_target_y - self.cur_y)
            d_traveled = math.hypot(self.cur_x - self.direct_start_x, self.cur_y - self.direct_start_y)
            
            # 5cm以内または目標距離にほぼ到達した場合は旋回フェーズへ
            if d_remain <= 0.05 or d_traveled >= (self.direct_total_dist - 0.03):
                self.get_logger().info("[Direct Drive] Move phase completed. Switching to turn phase.")
                self.direct_phase = 'turn'
            else:
                # 開始時に記録した固定直線角度に向かって直進
                yaw_error = self._normalize_angle(self.direct_line_angle - self.cur_yaw)
                
                # 前進速度 (最大 0.15 m/s, 残り距離に応じた減速)
                v_target = max(0.04, min(0.15, 0.4 * d_remain))
                
                # 開始時に決まった固定直線ベクトル
                vx_field = v_target * math.cos(self.direct_line_angle)
                vy_field = v_target * math.sin(self.direct_line_angle)
                
                # フィールド速度 -> ロボットローカル速度 (linear.y: 前後, linear.x: 左右)
                cmd.linear.y =  vx_field * math.cos(self.cur_yaw) + vy_field * math.sin(self.cur_yaw)
                cmd.linear.x = -vx_field * math.sin(self.cur_yaw) + vy_field * math.cos(self.cur_yaw)
                cmd.angular.z = max(-0.2, min(0.2, 0.8 * yaw_error))
                self.cmd_pub.publish(cmd)
                return

        if self.direct_phase == 'turn':
            yaw_error = self._normalize_angle(self.direct_target_yaw - self.cur_yaw)
            if not hasattr(self, 'direct_turn_count'):
                self.direct_turn_count = 0
            self.direct_turn_count += 1
            
            # 5度以内に到達、または3秒間(60ステップ)経過したら完了とみなす
            if abs(yaw_error) <= math.radians(5.0) or self.direct_turn_count > 60:
                self.get_logger().info(f"[Direct Drive] Turn completed (YawError={math.degrees(yaw_error):.1f}deg).")
                self.direct_turn_count = 0
                if self.control_timer:
                    self.control_timer.cancel()
                    self.control_timer = None
                self.cmd_pub.publish(Twist())  # 停止
                cb = self.direct_done_cb
                self.direct_done_cb = None
                if cb:
                    cb()
            else:
                cmd.angular.z = max(-0.15, min(0.15, 0.5 * yaw_error))
                self.cmd_pub.publish(cmd)

    def _nav_status_cb(self, msg):
        """fast_nav_node (C++ 超高速ナビ) からの自律移動ステータスを受信"""
        status = msg.data
        if status == "arrived":
            self.get_logger().info("FastNav arrived at goal successfully!")
            cb = self.nav_done_cb
            self.nav_done_cb = None
            if cb:
                cb()
        elif status in ("cancelled", "planning_failed"):
            self.get_logger().warn(f"FastNav status: {status}")
            self.nav_done_cb = None
            self._transition(STATE_IDLE)

    def send_nav2_goal(self, x, y, yaw, done_callback):
        self.get_logger().info(f"Sending FastNav goal: X={x:.3f}, Y={y:.3f}, Yaw={math.degrees(yaw):.2f}")
        self.nav_done_cb = done_callback

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y

        cy = math.cos(yaw / 2.0)
        sy = math.sin(yaw / 2.0)
        goal_msg.pose.orientation.w = cy
        goal_msg.pose.orientation.z = sy

        self.goal_pub.publish(goal_msg)

    def _stop(self):
        try:
            self.nav_done_cb = None
            if self.control_timer is not None:
                self.control_timer.cancel()
                self.control_timer = None

            if self.action_timer is not None:
                self.action_timer.cancel()
                self.action_timer = None

            self._transition(STATE_IDLE)
            self.navigating = False

            mode_msg = Bool()
            mode_msg.data = False
            self.mode_pub.publish(mode_msg)

            self.cmd_pub.publish(Twist())
        except Exception:
            pass

    def start_nav_point(self, x_mm, y_mm, yaw_deg=None, nav_type="nav2"):
        """指定座標 (mm, deg) への直接自動走行"""
        # If yaw_deg is None (not provided), keep current robot yaw
        if yaw_deg is None:
            yaw_rad = self.cur_yaw
        else:
            yaw_rad = math.radians(float(yaw_deg))

        self.current_preset_id = 99
        PRESET_LOCATIONS[99] = {
            "x": safe_x * 1000.0,
            "y": safe_y * 1000.0,
            "yaw": math.degrees(yaw_rad),
            "action_id": 1,
            "turn_yaw": math.degrees(yaw_rad),
            "action_mode": "auto",
            "nav_type": nav_type
        }

        mode_msg = Bool()
        mode_msg.data = True
        self.mode_pub.publish(mode_msg)

        self._transition(STATE_NAV_TO_GOAL)
        self.send_goal(safe_x, safe_y, yaw_rad, self._on_point_goal_reached)

        self.broadcast_to_ws({
            "type": "target_confirmed",
            "x": int(round(safe_x * 1000.0)),
            "y": int(round(safe_y * 1000.0)),
            "clamped": clamped,
            "clearance": int(round(cl * 1000.0))
        })

    def _on_point_goal_reached(self):
        self.get_logger().info("[Web Nav] Point navigation completed successfully.")
        self._transition(STATE_IDLE)
        self._stop()

    def start_nav_preset(self, loc_id):
        if loc_id not in PRESET_LOCATIONS:
            return
            
        self.current_preset_id = loc_id
        loc = PRESET_LOCATIONS[loc_id]
        
        mode_msg = Bool()
        mode_msg.data = True
        self.mode_pub.publish(mode_msg)
        
        self._transition(STATE_NAV_TO_GOAL)
        self.send_goal(loc['x'] / 1000.0, loc['y'] / 1000.0, math.radians(loc['yaw']), self._on_goal_reached)

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
        
        self.get_logger().info(f"Starting action sequence. Robot should already be at target angle: {turn_yaw_deg} deg.")
        
        # 既に到着時の角度(yaw)とアクション用角度(turn_yaw)を統合しているため、
        # 到着時点で目標角度は向いている。再度send_goalすると位置ズレを再修正しようとして
        # 「到着後にまた動く」原因になるため、ここでは直接CAN送信へ進む。
        self._on_turn_completed()

    def _on_turn_completed(self):
        loc = PRESET_LOCATIONS.get(self.current_preset_id, {})
        action_id = loc.get("action_id", 1)
        
        self.get_logger().info(f"========== [ACTION EXECUTION] Sending Action ID {action_id} to CAN ID 0x520 ==========")
        
        # 受信漏れを防ぐため 0x520 を 1ms間隔をあけて5回パブリッシュ
        can_msg = Int32MultiArray()
        can_msg.data = [0x520, action_id, 0, 0, 0, 0, 0, 0]
        for _ in range(5):
            self.can_pub.publish(can_msg)
            time.sleep(0.001)
        
        self.get_logger().info("Action triggered. Waiting 5 seconds...")
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
        
        self.send_goal(
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
                "x": float(data.get("x", 0.0)),
                "y": float(data.get("y", 0.0)),
                "action_id": data.get("action_id", 1),
                "turn_yaw": data.get("turn_yaw", 0.0),
                "nav_type": data.get("nav_type", "nav2"),
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
                        "auto_mode": bool(_node.navigating),
                        "map_info": {
                            "width": float(_node.map_width),
                            "height": float(_node.map_height),
                            "resolution": float(_node.map_resolution),
                            "origin_x": float(_node.map_origin_x),
                            "origin_y": float(_node.map_origin_y),
                            "image": str(_node.map_image_name)
                        }
                    }))
                except Exception:
                    break
            await asyncio.sleep(0.05)

    asyncio.create_task(send_status())
    try:
        async for message in websocket:
            cmd = json.loads(message)
            action = cmd.get("action")
            if action == "navigate_preset" and _node:
                _node.start_nav_preset(cmd.get("id"))
            elif action == "navigate_point" and _node:
                x = float(cmd.get("x", 0.0))
                y = float(cmd.get("y", 0.0))
                # yaw is optional; if not provided, keep current robot yaw
                if "yaw" in cmd:
                    yaw = float(cmd["yaw"])
                else:
                    yaw = None
                nav_type = str(cmd.get("nav_type", "nav2"))
                _node.start_nav_point(x, y, yaw, nav_type)
            elif action == "check_target" and _node:
                raw_x = float(cmd.get("x", 0.0)) / 1000.0
                raw_y = float(cmd.get("y", 0.0)) / 1000.0
                safe_x, safe_y, cl, clamped = _node.adjust_goal_for_hitbox(raw_x, raw_y)
                await websocket.send(json.dumps({
                    "type": "target_checked",
                    "orig_x": int(cmd.get("x", 0.0)),
                    "orig_y": int(cmd.get("y", 0.0)),
                    "x": int(round(safe_x * 1000.0)),
                    "y": int(round(safe_y * 1000.0)),
                    "clearance": int(round(cl * 1000.0)),
                    "clamped": clamped
                }))
            elif action == "gamepad" and _node:
                axes = cmd.get("axes", [])
                buttons = cmd.get("buttons", [])
                joy = Joy()
                joy.header.stamp = _node.get_clock().now().to_msg()
                joy.axes = [float(a) for a in axes]
                joy.buttons = [int(b) for b in buttons]
                _node.ps4_pub.publish(joy)
            elif action == "stop" and _node:
                _node._stop()
            elif action == "execute_action" and _node:
                if _node.state == STATE_ACTION:
                    _node.get_logger().info("Manual action execution trigger received from UI.")
                    _node._start_action_sequence()
            elif action == "update_presets" and _node:
                presets_data = cmd.get("presets", {})
                for pid_str, data in presets_data.items():
                    pid = int(pid_str)
                    if pid in PRESET_LOCATIONS:
                        if "x" in data and "y" in data:
                            raw_x = float(data["x"]) / 1000.0
                            raw_y = float(data["y"]) / 1000.0
                            safe_x, safe_y, _, _ = _node.adjust_goal_for_hitbox(raw_x, raw_y)
                            PRESET_LOCATIONS[pid]["x"] = round(safe_x * 1000.0)
                            PRESET_LOCATIONS[pid]["y"] = round(safe_y * 1000.0)
                        PRESET_LOCATIONS[pid]["action_id"] = int(data.get("action_id", 1))
                        PRESET_LOCATIONS[pid]["turn_yaw"] = float(data.get("turn_yaw", 0.0))
                        PRESET_LOCATIONS[pid]["yaw"] = float(data.get("turn_yaw", 0.0)) # 統一して変更
                        PRESET_LOCATIONS[pid]["nav_type"] = data.get("nav_type", "nav2")
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
        if self.path.startswith("/map.png"):
            import os
            from ament_index_python.packages import get_package_share_directory
            map_name = getattr(_node, "map_image_name", "map_red.png") if _node else "map_red.png"
            
            file_path = None
            try:
                pkg_share = get_package_share_directory('honrobo_pkg')
                file_path = os.path.join(pkg_share, 'map', map_name)
            except Exception:
                pass
                
            if not file_path or not os.path.exists(file_path):
                # ワークスペースのローカル相対パスを試みる
                local_path = os.path.join(os.getcwd(), 'src/honrobo_pkg/map', map_name)
                if os.path.exists(local_path):
                    file_path = local_path
                else:
                    # 最終的な絶対パスフォールバック
                    file_path = os.path.join("/home/haru/Documents/honrobo_2026/src/honrobo_pkg/map", map_name)

            if file_path and os.path.exists(file_path):
                self.send_response(200)
                self.send_header("Content-type", "image/png")
                self.end_headers()
                with open(file_path, "rb") as f:
                    self.wfile.write(f.read())
                return
            else:
                self.send_error(404, "Map image not found")
                return

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
