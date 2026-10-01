"""
nav_gui_node.py — 自律移動 監視・可視化GUIノード

仕様:
・目標進路の描画 (/plan)
・ロボットの現在位置および正面を矢印で明示 (/odom)
・マップ描画 (/map または YAMLファイル直接読込)
・インタラクティブな目標地点送信 (マップクリック -> /goal_pose)
・緊急停止 (スペースキー / 右クリック)
・ズーム (ホイール) / パン (ドラッグ) / リセット (Rキー)
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
import pygame
import math
import time
import os
import sys
import yaml
import threading


class NavGuiNode(Node):
    def __init__(self):
        super().__init__('nav_gui_node')

        # パラメータ
        self.declare_parameter('map_yaml', '')
        map_yaml_param = self.get_parameter('map_yaml').value or ''

        # ロボット向きオフセット (オドメトリ正面0度基準: デフォルト 0.0度)
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.yaw_offset_deg = float(self.get_parameter('yaw_offset_deg').value)

        # 画面座標系回転符号 (False: 画面Y軸下向き反転 -sin により反時計回りが画面上向き)
        self.declare_parameter('clockwise_yaw', False)
        self.clockwise_yaw = bool(self.get_parameter('clockwise_yaw').value)

        # ロボット実機寸法 (実機: 950mm x 950mm, ホイール実効半径: 350mm, 衝突判定半径: 480mm)
        self.declare_parameter('robot_width', 0.95)       # 車幅 (m)
        self.declare_parameter('robot_length', 0.95)      # 全長 (m)
        self.declare_parameter('wheel_dist', 0.35)        # 中心からホイールまでの実効距離 (m)
        self.declare_parameter('collision_radius', 0.48)  # 衝突判定半径 (m)
        self.robot_width = float(self.get_parameter('robot_width').value)
        self.robot_length = float(self.get_parameter('robot_length').value)
        self.wheel_dist = float(self.get_parameter('wheel_dist').value)
        self.collision_radius = float(self.get_parameter('collision_radius').value)

        # TF サブスクライバ設定
        import tf2_ros
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # サブスクライバー
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)
        self.create_subscription(Path, 'plan', self._plan_cb, 10)
        self.create_subscription(OccupancyGrid, 'map', self._map_cb, 10)
        self.create_subscription(OccupancyGrid, 'costmap', self._costmap_cb, 10)
        from rclpy.qos import qos_profile_sensor_data
        self.create_subscription(LaserScan, 'scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(String, 'nav_status', self._status_cb, 10)
        self.create_subscription(Bool, 'auto_mode', self._mode_cb, 10)

        # パブリッシャー
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.mode_pub = self.create_publisher(Bool, 'auto_mode', 10)
        self.cmd_pub = self.create_publisher(Twist, 'nav_cmd', 10)

        # ロボット状態
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_yaw = 0.0
        self.has_odom = False
        self.odom_count = 0
        self.trail = []  # 走行軌跡 [(x, y), ...]
        self.scan_pts = [] # LiDAR点群 (ローカル座標系)

        # 経路・目標状態
        self.planned_path = []  # [(x, y), ...]
        self.target_goal = None  # (gx, gy)
        self.nav_status = "IDLE"
        self.auto_mode = True

        # マップデータ
        self.map_w = 0
        self.map_h = 0
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.map_surf = None
        self.map_loaded = False
        self.costmap_surf = None
        self.show_costmap = False
        self.dist_map = None
        self.lock = threading.Lock()

        # マップの事前読み込み試行
        self._load_fallback_map(map_yaml_param)

    def _load_fallback_map(self, map_yaml_param):
        """トピック受信前でもマップファイルから直接即時表示"""
        candidates = []
        if map_yaml_param and os.path.exists(map_yaml_param):
            candidates.append(map_yaml_param)

        # ワークスペース内の既知マップファイル
        ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
        pkg_map_dir = os.path.join(ws_dir, 'src', 'honrobo_pkg', 'map')
        candidates.extend([
            os.path.join(pkg_map_dir, 'map_red.yaml'),
            os.path.join(pkg_map_dir, 'map_blue.yaml'),
            os.path.join(pkg_map_dir, 'map_test.yaml'),
            os.path.join(pkg_map_dir, 'map.yaml'),
        ])

        for ypath in candidates:
            if os.path.exists(ypath):
                try:
                    with open(ypath, 'r') as f:
                        meta = yaml.safe_load(f)
                    mdir = os.path.dirname(ypath)
                    img_path = os.path.join(mdir, meta.get('image', ''))
                    if os.path.exists(img_path):
                        self._build_map_surface_from_image(img_path, meta)
                        self.get_logger().info(f"Loaded initial map from {ypath}")
                        break
                except Exception as e:
                    self.get_logger().warn(f"Failed to load map {ypath}: {e}")

    def _build_map_surface_from_image(self, img_path, meta):
        try:
            raw_img = pygame.image.load(img_path)
            w, h = raw_img.get_size()
            with self.lock:
                self.map_w = w
                self.map_h = h
                self.resolution = float(meta.get('resolution', 0.05))
                origin = meta.get('origin', [0.0, 0.0, 0.0])
                self.origin_x = float(origin[0])
                self.origin_y = float(origin[1])
                # 画像のY軸反転は描画座標変換で吸収
                if pygame.display.get_init() and pygame.display.get_surface():
                    self.map_surf = raw_img.convert()
                else:
                    self.map_surf = raw_img
                self.map_loaded = True

                # 障害物距離マップの事前計算 (当たり判定用)
                try:
                    import cv2
                    import numpy as np
                    cv_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                    if cv_img is not None:
                        cv_ros = cv2.flip(cv_img, 0)
                        bin_grid = np.where(cv_ros < 50, 0, 255).astype(np.uint8)
                        self.dist_map = cv2.distanceTransform(bin_grid, cv2.DIST_L2, 5) * self.resolution
                except Exception as ex:
                    self.get_logger().warn(f"Distance map calculation failed: {ex}")
        except Exception as e:
            self.get_logger().error(f"Image load error: {e}")

    def _map_cb(self, msg: OccupancyGrid):
        with self.lock:
            self.map_w = msg.info.width
            self.map_h = msg.info.height
            self.resolution = float(msg.info.resolution)
            self.origin_x = float(msg.info.origin.position.x)
            self.origin_y = float(msg.info.origin.position.y)

            # OccupancyGrid から Pygame Surface を生成
            surf = pygame.Surface((self.map_w, self.map_h))
            pixels = pygame.PixelArray(surf)

            try:
                import numpy as np
                bin_grid = np.ones((self.map_h, self.map_w), dtype=np.uint8) * 255
            except ImportError:
                bin_grid = None

            for r in range(self.map_h):
                for c in range(self.map_w):
                    # ROSのグリッドは左下が原点(gy=0)
                    gy = r
                    gx = c
                    idx = gy * self.map_w + gx
                    val = msg.data[idx] if idx < len(msg.data) else -1
                    # 画面画像では上側がr=0
                    img_y = (self.map_h - 1) - gy
                    if val == 0:
                        color = (245, 247, 250)  # 走行可能 (白系)
                    elif val > 50:
                        color = (30, 41, 59)     # 障害物・壁 (濃紺黒)
                        if bin_grid is not None:
                            bin_grid[gy, gx] = 0
                    else:
                        color = (148, 163, 184)  # 未知 (灰色)
                    pixels[gx, img_y] = surf.map_rgb(color)
            del pixels
            self.map_surf = surf
            self.map_loaded = True

            # 障害物距離マップの計算 (当たり判定用)
            if bin_grid is not None:
                try:
                    import cv2
                    self.dist_map = cv2.distanceTransform(bin_grid, cv2.DIST_L2, 5) * self.resolution
                except Exception:
                    self.dist_map = None

    def _costmap_cb(self, msg: OccupancyGrid):
        try:
            csurf = pygame.Surface((msg.info.width, msg.info.height), pygame.SRCALPHA)
            for r in range(msg.info.height):
                for c in range(msg.info.width):
                    gy = r
                    gx = c
                    idx = gy * msg.info.width + gx
                    val = msg.data[idx] if idx < len(msg.data) else 0
                    img_y = (msg.info.height - 1) - gy
                    if val >= 100:
                        # 進入不可 (Lethal Hitbox): 濃い半透明赤
                        csurf.set_at((gx, img_y), (239, 68, 68, 130))
                    elif val > 0:
                        # 接近警戒 (Inflation Gradient): 半透明オレンジ
                        alpha = min(90, int(val * 1.2))
                        csurf.set_at((gx, img_y), (245, 158, 11, alpha))
            with self.lock:
                self.costmap_surf = csurf
        except Exception as e:
            self.get_logger().warn(f"Costmap surface error: {e}")

    def _odom_cb(self, msg: Odometry):
        self.odom_count += 1
        
        # TFによるAMCL補正 (map -> base_link) の取得を試みる
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
        except Exception as e:
            # AMCLが起動していない場合は、生オドメトリ(msg)を使用するフォールバック
            self.get_logger().warn(f"TF map->base_link failed, falling back to odom: {e}", throttle_duration_sec=5.0)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

        with self.lock:
            self.cur_x = x
            self.cur_y = y
            self.cur_yaw = yaw
            self.has_odom = True

            # 軌跡保存 (10cm 移動ごとに記録、40cm以上のワープ/移動は分割)
            last_pt = next((p for p in reversed(self.trail) if p is not None), None)
            if last_pt is None:
                self.trail.append((x, y))
            else:
                jump_d = math.hypot(x - last_pt[0], y - last_pt[1])
                if jump_d > 0.4:
                    self.trail.append(None)  # ワープ時は線を途切れさせる
                    self.trail.append((x, y))
                elif jump_d > 0.08:
                    self.trail.append((x, y))

            if len(self.trail) > 500:
                self.trail.pop(0)

    def _scan_cb(self, msg: LaserScan):
        pts = []
        # LiDARの取り付け向きが180度逆のため補正 (+ math.pi)
        angle = msg.angle_min + math.pi
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                lx = r * math.cos(angle) + 0.475  # ロボット前方中点 (全長0.95mの半分) のオフセット
                ly = r * math.sin(angle)
                pts.append((lx, ly))
            angle += msg.angle_increment
        with self.lock:
            self.scan_pts = pts

    def _plan_cb(self, msg: Path):
        pts = []
        for p in msg.poses:
            pts.append((p.pose.position.x, p.pose.position.y))
        with self.lock:
            self.planned_path = pts
            if pts:
                self.target_goal = pts[-1]
                self.nav_status = "NAVIGATING"
                # 新しい経路を受信した時は過去の古い軌跡をクリアして新ルートの軌跡のみ描画
                self.trail = [(self.cur_x, self.cur_y)]

    def _status_cb(self, msg: String):
        with self.lock:
            st = msg.data.lower()
            if st == "arrived":
                self.nav_status = "ARRIVED"
            elif st == "cancelled":
                self.nav_status = "CANCELLED"
            elif st == "planning_failed":
                self.nav_status = "PLAN_FAIL"
            elif st == "navigating":
                self.nav_status = "NAVIGATING"
            else:
                self.nav_status = msg.data.upper()

    def _mode_cb(self, msg: Bool):
        with self.lock:
            self.auto_mode = msg.data

    def send_goal(self, gx, gy):
        """クリックした地点へ自律移動目標を送信"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(gx)
        msg.pose.position.y = float(gy)
        msg.pose.position.z = 0.0
        # 目標方向は現在のロボットの向きを維持
        with self.lock:
            cur_yaw = self.cur_yaw
        cy = math.cos(cur_yaw / 2.0)
        sy = math.sin(cur_yaw / 2.0)
        msg.pose.orientation.z = sy
        msg.pose.orientation.w = cy
        self.goal_pub.publish(msg)
        self.get_logger().info(f"Target Goal Sent via GUI: ({gx:.2f}, {gy:.2f})")

        with self.lock:
            self.target_goal = (gx, gy)
            self.nav_status = "PLANNING..."

    def emergency_stop(self):
        """緊急停止"""
        mode_msg = Bool()
        mode_msg.data = False
        self.mode_pub.publish(mode_msg)
        self.cmd_pub.publish(Twist())
        with self.lock:
            self.nav_status = "STOPPED"
            self.planned_path = []
            self.target_goal = None
        self.get_logger().warn("Emergency Stop Triggered via GUI!")


# ============================================================
# Pygame GUI メイン描画ループ
# ============================================================
def run_gui(node: NavGuiNode):
    pygame.init()
    pygame.font.init()

    win_w, win_h = 1000, 700
    screen = pygame.display.set_mode((win_w, win_h), pygame.RESIZABLE)
    pygame.display.set_caption("Honrobo Autonomous Navigation Monitor [FastNav GUI]")
    clock = pygame.time.Clock()

    # フォント準備
    font_large = pygame.font.SysFont("DejaVu Sans, Helvetica, Arial", 20, bold=True)
    font_mid = pygame.font.SysFont("DejaVu Sans, Helvetica, Arial", 14, bold=True)
    font_small = pygame.font.SysFont("DejaVu Sans, Helvetica, Arial", 12)
    font_hud = pygame.font.SysFont("DejaVu Sans Mono, Courier, monospace", 13)

    # ビューポート制御 (ワールド座標 -> スクリーン座標)
    zoom = 85.0  # pixels per meter
    pan_x = 4.5  # 画面中央のワールドX (m)
    pan_y = -0.9 # 画面中央のワールドY (m)
    follow_robot = False
    is_dragging = False
    drag_start = (0, 0)
    drag_pan_start = (pan_x, pan_y)

    running = True
    pulse_phase = 0.0

    while running and rclpy.ok():
        dt = clock.tick(60) / 1000.0
        pulse_phase = (pulse_phase + dt * 4.0) % (2.0 * math.pi)

        # ── 1. イベント処理 ──
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break
            elif event.type == pygame.VIDEORESIZE:
                win_w, win_h = event.w, event.h
                screen = pygame.display.set_mode((win_w, win_h), pygame.RESIZABLE)

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # 左クリック: 目標地点送信 または ドラッグ開始
                    # HUD上部(40px)および下部(30px)以外がマップ領域
                    if 45 <= event.pos[1] <= win_h - 35:
                        # スクリーン座標 -> ワールド座標
                        wx = pan_x + (event.pos[0] - win_w / 2.0) / zoom
                        wy = pan_y - (event.pos[1] - win_h / 2.0) / zoom
                        node.send_goal(wx, wy)
                elif event.button == 3:  # 右クリック: ドラッグ移動開始 または 非常停止
                    is_dragging = True
                    drag_start = event.pos
                    drag_pan_start = (pan_x, pan_y)
                elif event.button == 4:  # ホイール上: ズームイン
                    mx, my = event.pos
                    wx_before = pan_x + (mx - win_w / 2.0) / zoom
                    wy_before = pan_y - (my - win_h / 2.0) / zoom
                    zoom = min(zoom * 1.15, 300.0)
                    pan_x = wx_before - (mx - win_w / 2.0) / zoom
                    pan_y = wy_before + (my - win_h / 2.0) / zoom
                elif event.button == 5:  # ホイール下: ズームアウト
                    mx, my = event.pos
                    wx_before = pan_x + (mx - win_w / 2.0) / zoom
                    wy_before = pan_y - (my - win_h / 2.0) / zoom
                    zoom = max(zoom / 1.15, 20.0)
                    pan_x = wx_before - (mx - win_w / 2.0) / zoom
                    pan_y = wy_before + (my - win_h / 2.0) / zoom

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 3:
                    is_dragging = False

            elif event.type == pygame.MOUSEMOTION:
                if is_dragging:
                    dx = event.pos[0] - drag_start[0]
                    dy = event.pos[1] - drag_start[1]
                    pan_x = drag_pan_start[0] - dx / zoom
                    pan_y = drag_pan_start[1] + dy / zoom

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    node.emergency_stop()
                elif event.key == pygame.K_r:
                    # ビューのリセット
                    with node.lock:
                        if node.map_loaded:
                            pan_x = node.origin_x + (node.map_w * node.resolution) / 2.0
                            pan_y = node.origin_y + (node.map_h * node.resolution) / 2.0
                        else:
                            pan_x, pan_y = 0.0, 0.0
                    zoom = 85.0
                    follow_robot = False
                elif event.key == pygame.K_f:
                    follow_robot = not follow_robot
                elif event.key == pygame.K_c:
                    with node.lock:
                        node.show_costmap = not node.show_costmap
                    node.get_logger().info(f"Costmap Hitbox Overlay: {node.show_costmap}")
                elif event.key == pygame.K_t:
                    with node.lock:
                        node.trail.clear()
                    node.get_logger().info("Trail cleared via GUI shortcut.")
                elif event.key == pygame.K_o:
                    # Oキー: 向きオフセットを90度右回転 (-90度)
                    with node.lock:
                        node.yaw_offset_deg = (node.yaw_offset_deg - 90.0) % 360.0
                        if node.yaw_offset_deg > 180.0:
                            node.yaw_offset_deg -= 360.0
                        node.get_logger().info(f"Yaw Display Offset adjusted: {node.yaw_offset_deg:.0f}°")
                elif event.key == pygame.K_i:
                    # Iキー: 回転方向の反転 (時計回り/反時計回りの切り替え)
                    with node.lock:
                        node.clockwise_yaw = not node.clockwise_yaw
                        node.get_logger().info(f"Yaw Rotation Inverted: Clockwise={node.clockwise_yaw}")

        # ロボット追従
        if follow_robot and node.has_odom:
            pan_x = node.cur_x
            pan_y = node.cur_y

        # ── 2. ワールド -> スクリーン座標変換関数 ──
        def w2s(wx, wy):
            sx = int(win_w / 2.0 + (wx - pan_x) * zoom)
            sy = int(win_h / 2.0 - (wy - pan_y) * zoom)
            return sx, sy

        # 背景塗りつぶし (ダークテーマ)
        screen.fill((15, 23, 42))  # slate-900

        # ── 3. マップ描画 ──
        with node.lock:
            map_surf = node.map_surf
            map_w = node.map_w
            map_h = node.map_h
            res = node.resolution
            ox = node.origin_x
            oy = node.origin_y
            map_ok = node.map_loaded

        if map_ok and map_surf:
            # マップ画像全体のワールド座標外枠
            # 左下: (ox, oy), 右上: (ox + map_w * res, oy + map_h * res)
            sx_left, sy_bottom = w2s(ox, oy)
            sx_right, sy_top = w2s(ox + map_w * res, oy + map_h * res)
            target_w = max(1, sx_right - sx_left)
            target_h = max(1, sy_bottom - sy_top)

            if target_w > 0 and target_h > 0 and sx_right > 0 and sx_left < win_w and sy_bottom > 0 and sy_top < win_h:
                try:
                    scaled_map = pygame.transform.scale(map_surf, (target_w, target_h))
                    screen.blit(scaled_map, (sx_left, sy_top))

                    # コストマップ (当たり判定・衝突不可領域オーバーレイ)
                    with node.lock:
                        show_c = node.show_costmap
                        cost_s = node.costmap_surf
                    if show_c and cost_s is not None:
                        scaled_cost = pygame.transform.scale(cost_s, (target_w, target_h))
                        screen.blit(scaled_cost, (sx_left, sy_top))
                except Exception:
                    pass

        # ── 4. メートルグリッド線 & 座標軸描画 ──
        # 1m ごとのグリッド線
        grid_min_x = math.floor(pan_x - (win_w / 2.0) / zoom) - 1
        grid_max_x = math.ceil(pan_x + (win_w / 2.0) / zoom) + 1
        grid_min_y = math.floor(pan_y - (win_h / 2.0) / zoom) - 1
        grid_max_y = math.ceil(pan_y + (win_h / 2.0) / zoom) + 1

        for gx in range(int(grid_min_x), int(grid_max_x) + 1):
            sx, _ = w2s(gx, 0)
            if 0 <= sx <= win_w:
                col = (51, 65, 85) if gx != 0 else (148, 163, 184)
                pygame.draw.line(screen, col, (sx, 0), (sx, win_h), 1 if gx != 0 else 2)
                # メートル目盛りラベル
                if gx % 1 == 0:
                    lbl = font_small.render(f"{gx}m", True, (100, 116, 139))
                    screen.blit(lbl, (sx + 3, win_h - 52))

        for gy in range(int(grid_min_y), int(grid_max_y) + 1):
            _, sy = w2s(0, gy)
            if 0 <= sy <= win_h:
                col = (51, 65, 85) if gy != 0 else (148, 163, 184)
                pygame.draw.line(screen, col, (0, sy), (win_w, sy), 1 if gy != 0 else 2)
                if gy % 1 == 0:
                    lbl = font_small.render(f"{gy}m", True, (100, 116, 139))
                    screen.blit(lbl, (6, sy - 14))

        # ── 5. ロボット走行履歴 (トレイル: 温かみのあるアンバー色) ──
        with node.lock:
            trail_pts = list(node.trail)
        cur_seg = []
        for pt in trail_pts:
            if pt is None:
                if len(cur_seg) >= 2:
                    s_seg = [w2s(tx, ty) for tx, ty in cur_seg]
                    pygame.draw.lines(screen, (245, 158, 11), False, s_seg, 2)
                cur_seg = []
            else:
                cur_seg.append(pt)
        if len(cur_seg) >= 2:
            s_seg = [w2s(tx, ty) for tx, ty in cur_seg]
            pygame.draw.lines(screen, (245, 158, 11), False, s_seg, 2)

        # ── 6. 目標進路 (Planned Path) の描画 ──
        with node.lock:
            path_pts = list(node.planned_path)
            goal_pos = node.target_goal

        if len(path_pts) >= 2:
            s_path = [w2s(px, py) for px, py in path_pts]

            # 外側のグロー線 (半透明風の太線)
            pygame.draw.lines(screen, (6, 182, 212), False, s_path, 5)
            # 内側のコア線 (明るいシアン)
            pygame.draw.lines(screen, (165, 243, 252), False, s_path, 2)

            # 各ウェイポイントのドット描画
            for i, spt in enumerate(s_path):
                if i == 0 or i == len(s_path) - 1:
                    continue
                pygame.draw.circle(screen, (6, 182, 212), spt, 4)
                pygame.draw.circle(screen, (255, 255, 255), spt, 2)

        # ── 7. 目標地点 (Goal Marker) 描画 ──
        if goal_pos:
            gx, gy = goal_pos
            gsx, gsy = w2s(gx, gy)
            pulse_r = int(12 + 4 * math.sin(pulse_phase))
            # 同心円パルス
            pygame.draw.circle(screen, (239, 68, 68), (gsx, gsy), pulse_r, 2)
            pygame.draw.circle(screen, (248, 113, 113), (gsx, gsy), 6)
            pygame.draw.circle(screen, (255, 255, 255), (gsx, gsy), 2)
            # ラベル
            g_lbl = font_mid.render(f"GOAL ({gx:.2f}, {gy:.2f})", True, (254, 202, 202))
            screen.blit(g_lbl, (gsx + 14, gsy - 10))

        # ── 8. 実機寸法ロボット本体 & 当たり判定 (Hitbox) & 正面方向矢印 ──
        with node.lock:
            rx, ry, ryaw = node.cur_x, node.cur_y, node.cur_yaw
            has_pos = node.has_odom
            yaw_offset = math.radians(node.yaw_offset_deg)
            is_cw = node.clockwise_yaw
            r_w = node.robot_width        # 0.95m
            r_l = node.robot_length       # 0.95m
            wheel_d = node.wheel_dist     # 0.35m
            col_r = node.collision_radius # 0.48m
            dist_map = node.dist_map
            map_res = node.resolution
            map_ox = node.origin_x
            map_oy = node.origin_y
            map_w_val = node.map_w
            map_h_val = node.map_h
            map_is_loaded = node.map_loaded

        # ROS REP-103標準Yaw角 (反時計回り+, 時計回り-) からスクリーン描画角への変換
        gui_yaw = ryaw + yaw_offset
        rot_sign = 1.0 if is_cw else -1.0
        cos_y = math.cos(gui_yaw)
        sin_y = math.sin(gui_yaw)

        def to_screen_pt(lx, ly, csx, csy):
            # lx: 前方(+), ly: 左方(+)
            dx = (lx * cos_y - ly * sin_y) * zoom
            dy = (lx * sin_y + ly * cos_y) * rot_sign * zoom
            return int(csx + dx), int(csy + dy)

        def draw_robot_chassis(csx, csy, is_active=True):
            hl = r_l / 2.0
            hw = r_w / 2.0

            # (1) 当たり判定エンベロープ (Collision Hitbox Circle: 半径 0.48m)
            col_px = max(10, int(col_r * zoom))
            hitbox_color = (244, 63, 94) if is_active else (71, 85, 105)
            pygame.draw.circle(screen, hitbox_color, (csx, csy), col_px, 1)

            # (2) 実機寸法の正方形シャーシ (0.95m x 0.95m)
            pt_fl = to_screen_pt(+hl, +hw, csx, csy)
            pt_fr = to_screen_pt(+hl, -hw, csx, csy)
            pt_rr = to_screen_pt(-hl, -hw, csx, csy)
            pt_rl = to_screen_pt(-hl, +hw, csx, csy)
            chassis_poly = [pt_fl, pt_fr, pt_rr, pt_rl]

            if is_active:
                pygame.draw.polygon(screen, (30, 41, 59), chassis_poly)        # シャーシ内側
                pygame.draw.polygon(screen, (56, 189, 248), chassis_poly, 2)  # 外枠輪郭
                # 前面バンパーを太いアンバー色で強調 (進行方向正面)
                pygame.draw.line(screen, (245, 158, 11), pt_fl, pt_fr, 4)
            else:
                pygame.draw.polygon(screen, (51, 65, 85), chassis_poly, 1)
                pygame.draw.line(screen, (100, 116, 139), pt_fl, pt_fr, 2)

            # (3) 4輪オムニホイールの実位置 (中心から 0.35m, 45°/135°/225°/315°)
            wheel_rad_px = max(4, int(0.06 * zoom))
            for ang_deg in [45, 135, 225, 315]:
                wa_rad = math.radians(ang_deg)
                w_lx = wheel_d * math.cos(wa_rad)
                w_ly = wheel_d * math.sin(wa_rad)
                wsx, wsy = to_screen_pt(w_lx, w_ly, csx, csy)
                if is_active:
                    pygame.draw.circle(screen, (100, 116, 139), (wsx, wsy), wheel_rad_px)
                    pygame.draw.circle(screen, (203, 213, 225), (wsx, wsy), wheel_rad_px, 1)
                else:
                    pygame.draw.circle(screen, (71, 85, 105), (wsx, wsy), wheel_rad_px, 1)

            # (4) 正面方向矢印 (Heading Arrow)
            arrow_len = hl * 1.55
            tip_x, tip_y = to_screen_pt(arrow_len, 0.0, csx, csy)
            if is_active:
                pygame.draw.line(screen, (245, 158, 11), (csx, csy), (tip_x, tip_y), 3)
                head_len = max(8, int(0.18 * zoom))
                wing_w = max(5, int(0.09 * zoom))
                w1 = to_screen_pt(arrow_len - head_len / zoom, wing_w / zoom, csx, csy)
                w2 = to_screen_pt(arrow_len - head_len / zoom, -wing_w / zoom, csx, csy)
                arrow_poly = [(tip_x, tip_y), w1, w2]
                pygame.draw.polygon(screen, (245, 158, 11), arrow_poly)
                pygame.draw.polygon(screen, (254, 240, 138), arrow_poly, 1)
                # 中心LED
                pygame.draw.circle(screen, (96, 165, 250), (csx, csy), 4)

            # (5) LiDAR点群の描画
            with node.lock:
                scan_points = node.scan_pts[:]
            if scan_points:
                # 画面上に点をプロット
                cyaw = node.cur_yaw
                cc = math.cos(cyaw)
                ss = math.sin(cyaw)
                sw = screen.get_width()
                sh = screen.get_height()
                for lx, ly in scan_points:
                    # ロボット座標(lx, ly) からワールド座標へ
                    wx = rx + (lx * cc - ly * ss)
                    wy = ry + (lx * ss + ly * cc)
                    # ワールド座標からスクリーン座標へ
                    sx, sy = w2s(wx, wy)
                    # 画面内チェック (不要な描画を省く)
                    if 0 <= sx <= sw and 0 <= sy <= sh:
                        pygame.draw.circle(screen, (134, 239, 172), (int(sx), int(sy)), 2)
            else:
                pygame.draw.line(screen, (100, 116, 139), (csx, csy), (tip_x, tip_y), 2)

        if has_pos:
            rsx, rsy = w2s(rx, ry)
            draw_robot_chassis(rsx, rsy, is_active=True)

            # ロボット座標および実機寸法ラベル
            deg_val = math.degrees(gui_yaw) % 360.0
            pos_tag = font_small.render(f"({rx:.2f}, {ry:.2f}) {deg_val:.0f}° [0.95×0.95m Hitbox: {col_r*2*1000:.0f}mm]", True, (226, 232, 240))
            col_px = max(10, int(col_r * zoom))
            screen.blit(pos_tag, (rsx - pos_tag.get_width() // 2, rsy + col_px + 6))
        else:
            # 未受信時でもスタート位置 (0, 0) にプレビュー描画
            rsx, rsy = w2s(0.0, 0.0)
            draw_robot_chassis(rsx, rsy, is_active=False)
            warn_lbl = font_small.render("[WAITING FOR /odom] 0.95m×0.95m", True, (248, 113, 113))
            col_px = max(10, int(col_r * zoom))
            screen.blit(warn_lbl, (rsx - warn_lbl.get_width() // 2, rsy + col_px + 6))

        # ── 8.5 マウスカーソル位置のリアルタイム当たり判定プレビュー ──
        mouse_pos = pygame.mouse.get_pos()
        if 45 <= mouse_pos[1] <= win_h - 35 and not is_dragging:
            cur_mx, cur_my = mouse_pos
            mwx = pan_x + (cur_mx - win_w / 2.0) / zoom
            mwy = pan_y - (cur_my - win_h / 2.0) / zoom

            # 障害物との距離判定
            is_col = False
            clearance_m = 999.0
            if dist_map is not None and map_is_loaded:
                mgx = int(round((mwx - map_ox) / map_res))
                mgy = int(round((mwy - map_oy) / map_res))
                if 0 <= mgx < map_w_val and 0 <= mgy < map_h_val:
                    clearance_m = float(dist_map[mgy, mgx])
                    if clearance_m <= col_r:
                        is_col = True
                else:
                    is_col = True
                    clearance_m = 0.0

            # ゴーストフットプリント描画 (半透明)
            ghost_surf = pygame.Surface((win_w, win_h), pygame.SRCALPHA)
            gh_hl = r_l / 2.0
            gh_hw = r_w / 2.0
            def ghost_to_screen(lx, ly):
                dx = (lx * cos_y - ly * sin_y) * zoom
                dy = (lx * sin_y + ly * cos_y) * rot_sign * zoom
                return int(cur_mx + dx), int(cur_my + dy)

            g_poly = [
                ghost_to_screen(+gh_hl, +gh_hw),
                ghost_to_screen(+gh_hl, -gh_hw),
                ghost_to_screen(-gh_hl, -gh_hw),
                ghost_to_screen(-gh_hl, +gh_hw)
            ]
            ghost_fill = (239, 68, 68, 80) if is_col else (34, 197, 94, 70)
            ghost_line = (239, 68, 68, 200) if is_col else (34, 197, 94, 180)
            pygame.draw.polygon(ghost_surf, ghost_fill, g_poly)
            pygame.draw.polygon(ghost_surf, ghost_line, g_poly, 2)
            # 衝突判定円プレビュー
            g_col_px = max(10, int(col_r * zoom))
            pygame.draw.circle(ghost_surf, ghost_line, (cur_mx, cur_my), g_col_px, 1)
            screen.blit(ghost_surf, (0, 0))

            # 判定バッジ
            if is_col:
                tip_txt = f"⚠️ 衝突判定: 障害物まで {clearance_m*1000:.0f}mm (当たり判定 {col_r*1000:.0f}mm)"
                tip_surf = font_small.render(tip_txt, True, (254, 202, 202))
                tip_bg = pygame.Rect(cur_mx + 12, cur_my + 10, tip_surf.get_width() + 10, 22)
                pygame.draw.rect(screen, (153, 27, 27, 230), tip_bg, border_radius=4)
                pygame.draw.rect(screen, (239, 68, 68), tip_bg, 1, border_radius=4)
                screen.blit(tip_surf, (cur_mx + 17, cur_my + 14))
            else:
                tip_txt = f"✔️ 目標可能 ({mwx:.2f}, {mwy:.2f}) 余裕: {clearance_m*1000:.0f}mm"
                tip_surf = font_small.render(tip_txt, True, (187, 247, 208))
                tip_bg = pygame.Rect(cur_mx + 12, cur_my + 10, tip_surf.get_width() + 10, 22)
                pygame.draw.rect(screen, (20, 83, 45, 230), tip_bg, border_radius=4)
                pygame.draw.rect(screen, (34, 197, 94), tip_bg, 1, border_radius=4)
                screen.blit(tip_surf, (cur_mx + 17, cur_my + 14))

        # ── 9. 上部 HUD ヘッダバー ──
        hud_bar_rect = pygame.Rect(0, 0, win_w, 42)
        pygame.draw.rect(screen, (2, 6, 23, 230), hud_bar_rect)
        pygame.draw.line(screen, (51, 65, 85), (0, 42), (win_w, 42), 1)

        # タイトル
        title_surf = font_large.render("FASTNAV 2D MONITOR", True, (56, 189, 248))
        screen.blit(title_surf, (15, 10))

        # ナビステータスバッジ
        with node.lock:
            nst = node.nav_status
            am = node.auto_mode
            cur_x_disp = node.cur_x
            cur_y_disp = node.cur_y
            tgt_g = node.target_goal

        if nst == "NAVIGATING":
            st_color = (34, 197, 94)  # green
        elif nst == "ARRIVED":
            st_color = (59, 130, 246) # blue
        elif nst == "STOPPED":
            st_color = (239, 68, 68)  # red
        else:
            st_color = (148, 163, 184) # slate

        st_badge = font_mid.render(f"[{nst}]", True, st_color)
        screen.blit(st_badge, (245, 12))

        mode_badge = font_mid.render("[AUTO]" if am else "[MANUAL]", True, (34, 197, 94) if am else (245, 158, 11))
        screen.blit(mode_badge, (360, 12))

        # 座標情報および横偏差 (CTE)
        deg_disp = math.degrees(gui_yaw) % 360.0
        coords_str = f"POS: X:{cur_x_disp:>+5.2f}m  Y:{cur_y_disp:>+5.2f}m  Yaw:{deg_disp:>4.0f}°"
        if tgt_g:
            dist_rem = math.hypot(tgt_g[0] - cur_x_disp, tgt_g[1] - cur_y_disp)
            coords_str += f" | DIST:{dist_rem:>4.2f}m"
        if len(path_pts) >= 2:
            # パスとの最近傍距離 (CTE: 横偏差) を計算
            min_cte = 1e9
            for i in range(len(path_pts) - 1):
                p1, p2 = path_pts[i], path_pts[i+1]
                dx, dy = p2[0] - p1[0], p2[1] - p1[1]
                l2 = dx*dx + dy*dy
                if l2 < 1e-6:
                    d = math.hypot(cur_x_disp - p1[0], cur_y_disp - p1[1])
                else:
                    t = max(0.0, min(1.0, ((cur_x_disp - p1[0]) * dx + (cur_y_disp - p1[1]) * dy) / l2))
                    proj_x = p1[0] + t * dx
                    proj_y = p1[1] + t * dy
                    d = math.hypot(cur_x_disp - proj_x, cur_y_disp - proj_y)
                if d < min_cte:
                    min_cte = d
            if min_cte < 1e8:
                coords_str += f" | CTE:{min_cte*100:>3.0f}cm"

        coords_surf = font_hud.render(coords_str, True, (241, 245, 249))
        screen.blit(coords_surf, (win_w - coords_surf.get_width() - 15, 13))

        # ── 10. 下部 操作ヘルパーバー ──
        btm_bar_rect = pygame.Rect(0, win_h - 30, win_w, 30)
        pygame.draw.rect(screen, (2, 6, 23, 230), btm_bar_rect)
        pygame.draw.line(screen, (51, 65, 85), (0, win_h - 30), (win_w, win_h - 30), 1)

        help_text = "[Click]: Goal (Hitbox Check) | [Space]: STOP | [C]: Costmap/Hitbox | [T]: Clear Trail | [O]: Offset 90° | [I]: Invert Yaw | [R]: Reset | [F]: Follow | [Wheel]: Zoom"
        help_surf = font_small.render(help_text, True, (148, 163, 184))
        screen.blit(help_surf, (15, win_h - 22))

        # ズーム率表示
        zoom_str = f"Zoom: {zoom:.0f} px/m"
        zoom_surf = font_small.render(zoom_str, True, (148, 163, 184))
        screen.blit(zoom_surf, (win_w - zoom_surf.get_width() - 15, win_h - 22))

        pygame.display.flip()

    pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    node = NavGuiNode()

    def _spin():
        try:
            rclpy.spin(node)
        except (ExternalShutdownException, Exception):
            pass

    # ROS 2 スピンをバックグラウンドスレッドで実行
    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    try:
        run_gui(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
