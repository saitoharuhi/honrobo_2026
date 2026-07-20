# Nav2自動運転システム 実装仕様書

本仕様書は、オドメトリとジャイロのみ（LiDAR/カメラなし）を搭載した3輪独立ステアリングロボットにおいて、Nav2を用いた自律移動、到着後のアクション実行（CAN送信）、および初期の向き（0度）への自動復帰シーケンスを実装するための詳細な仕様です。

他コーディングAIに本ドキュメントをそのまま読み込ませることで、実装コードおよび設定ファイルを漏れなく生成できます。

---

## 1. 全体構成と座標系 (TF) 定義

ROS 2およびNav2が自律移動を行うため、以下のTFツリーを構成します。

```text
map (地図の原点) 
  └── [静的TF] (x=0, y=0, yaw=0 固定)
      odom (自己位置の原点 = スタート地点)
        └── [動的TF] (zikoiti_node.pyが配信)
            base_link (ロボットの中心座標)
```

- **`map` → `odom`**:
  ロボットの起動位置を常にスタート地点 `(0, 0, 0)` とするため、位置ズレ・角度ズレを `0` として完全に一致させます。
  `static_transform_publisher` もしくは Launch ファイル内で静的TFを配信します。
- **`odom` → `base_link`**:
  `zikoiti_node.py` 内で、ジャイロとオドメトリセンサーから計算される真の位置 `(true_x, true_y, z_rad)` を動的TFとして配信します。

---

## 2. 各ノードの実装仕様

### ① `zikoiti_node.py` の修正仕様
オドメトリ情報のパブリッシュに加え、動的TFブロードキャスターを実装します。

- **依存パッケージの追加**: `tf2_ros` から `TransformBroadcaster`、`geometry_msgs.msg` から `TransformStamped` をインポート。
- **初期化 (`__init__`)**:
  - `self.tf_broadcaster = TransformBroadcaster(self)` を初期化。
- **更新処理 (`update`)**:
  `/odom` トピック（`Odometry`メッセージ）をパブリッシュするコードの直後に、以下のTF変換（`TransformStamped`）を配信する処理を追加します。
  - `header.stamp`: 現在のROS時間
  - `header.frame_id`: `'odom'`
  - `child_frame_id`: `'base_link'`
  - `transform.translation.x`: `self.true_x`
  - `transform.translation.y`: `self.true_y`
  - `transform.translation.z`: `0.0`
  - `transform.rotation`: `self._euler_to_quat(0, 0, z_rad)`

---

### ② `web_node.py` の修正仕様（自動運転制御＆WebSocket中継）
既存のP制御による自律移動を廃止し、**Nav2のアクションクライアント**を用いたシーケンス制御（ステートマシン）へ置き換えます。

#### 1. プリセットデータ構造の更新
目的地ごとに、移動目標座標、到着時の向き、およびマイコンに送る「動作番号」と「動作時間（タイマー秒数）」を定義します。
```python
PRESET_LOCATIONS = {
    # 例：1番の目的地 (X: 1.5m, Y: 2.0m, 到着時に90度を向く, 動作番号: 5, 所要時間: 4.5秒)
    1: {
        "x": 1.5,           # メートル単位
        "y": 2.0,           # メートル単位
        "yaw": math.radians(90.0), # ラジアン単位
        "action_id": 5,     # CANでマイコンに送る動作番号
        "wait_time": 4.5    # 動作完了待ちのタイマー秒数
    },
    2: {
        "x": 3.0, 
        "y": 1.0, 
        "yaw": math.radians(180.0),
        "action_id": 8,
        "wait_time": 3.0
    }
}
```

#### 2. シーケンス管理ステートマシン
`web_node.py` 内に以下の状態（State）を定義し、順次遷移させます。
- `STATE_IDLE`: 待機状態。
- `STATE_NAV_TO_GOAL`: 目的地へNav2で自律移動している状態。
- `STATE_ACTION`: 目的地に到着し、動作番号をCAN送信してタイマー待機している状態。
- `STATE_RETURN_TO_ZERO`: タイマーが満了し、同じ座標で角度を「0度」に戻すためにNav2でその場旋回させている状態。

#### 3. Nav2 アクションクライアントの実装
- `nav2_msgs.action` の `NavigateToPose` アクションを利用し、目標姿勢を送信します。
- **ゴールキャンセル処理**: 手動割り込みや緊急停止が要求された場合、`self.nav_to_pose_client.cancel_all_goals()` もしくはアクティブなゴールハンドルに対して `cancel_goal_async()` を送信し、Nav2の動作を即時停止させます。

#### 4. アクション実行（CAN送信）とタイマー制御
- `STATE_NAV_TO_GOAL` が `SUCCEEDED`（成功）で完了したことを検知すると、状態を `STATE_ACTION` に移行します。
- 対象プリセットの `action_id` を、トピック `/can_tx` (標準メッセージ形式 `std_msgs/Int32MultiArray`、データ配列: `[CAN_ID, byte0, byte1, ...]`) にパブリッシュします。
  - 例: `[0x530, action_id, 0, 0, 0, 0, 0, 0, 0]` (CAN ID `0x530` としてパブリッシュ)
- パブリッシュと同時に、指定された `wait_time` 秒数で作動するワンショットタイマー（ROSの `create_timer`）を起動します。
- タイマーが満了（コールバックが実行）したら、状態を `STATE_RETURN_TO_ZERO` に移行し、目的地座標 `(x, y)` はそのままで、`yaw` 角度のみ `0.0` とした目標を再度Nav2へ送信します。
- `STATE_RETURN_TO_ZERO` が成功で完了したら、状態を `STATE_IDLE` に戻します。

#### 5. WebSocket送信メッセージの追加
現在の状態遷移をリアルタイムでブラウザに送信するため、以下のメッセージ（JSON）をWebSocketクライアントへブロードキャストします。
```json
{"type": "nav_status", "state": "moving"}             // STATE_NAV_TO_GOAL時
{"type": "nav_status", "state": "executing_action"}   // STATE_ACTION時
{"type": "nav_status", "state": "returning_to_zero"}  // STATE_RETURN_TO_ZERO時
{"type": "nav_status", "state": "completed"}          // 全シーケンス完了時
```

---

### ③ `roboware_node.py` の修正仕様（安全停止とトピック接続）
- **トピックの接続**: Nav2が出力する目標速度トピック `/cmd_vel`（`geometry_msgs/Twist`）を、ロボットが受信するトピック `/nav_cmd` へリマップするか、または `roboware_node.py` 自体の購読トピック名を `/cmd_vel` に変更します。
- **緊急停止割り込みの強化**:
  自動運転中（`/auto_mode` が `True`）にPS4コントローラの操作を検知した場合、既存の「自ノードの停止（速度0送信）」に加え、**`/auto_mode` トピックに `False` をパブリッシュ**します。
  - `web_node.py` は、`/auto_mode` が `False` になったことを購読（または内部トリガー）で検知し、即座にNav2の動作（ゴール）をキャンセルします。これにより、手動介入時にNav2の自動走行が完全に停止し、コントローラでの手動操縦へ即座に切り替わります。

---

## 3. Nav2 設定ファイル (`nav2_params.yaml`) の仕様

LiDAR等のセンサーを使用しない、3輪独立ステアリング（全方向移動型）ロボットに最適化されたパラメータ設定です。

### ① フットプリント（ロボットの寸法）
最大サイズ制限 `1.2m × 1.2m`（可動部展開時）を考慮し、中心から前後左右に `0.6m` の正方形フットプリントを定義します。
```yaml
footprint: "[ [0.6, 0.6], [0.6, -0.6], [-0.6, -0.6], [-0.6, 0.6] ]"
```

### ② 全方向移動（ホロノミック）の設定 (DWB Controller)
Y軸方向（横スライド移動）の走行パターンを有効化します。
```yaml
controller_server:
  ros__parameters:
    Publisher:
      max_vel_x: 1.0       # 最大前進速度 (m/s)
      min_vel_x: -1.0      # 最大後退速度 (m/s)
      max_vel_y: 1.0       # 最大横スライド速度 (m/s) (Y軸移動を許可)
      min_vel_y: -1.0      # 最大横スライド速度 (m/s) (Y軸移動を許可)
      max_vel_theta: 0.15  # 最大旋回速度 (rad/s) (極めて小さく設定し、移動中の首振りを抑制)

    dwb_plugins::GPPLocalPlanner:
      vx_samples: 20
      vy_samples: 20       # Y軸方向の速度探索サンプル数を有効にする
      vtheta_samples: 5    # 旋回の探索サンプル数を抑え、直進・横スライドを優先させる
```

### ③ コストマップのレイヤー設定
LiDARなどのセンサーがないため、障害物検出レイヤー（`obstacle_layer`）を無効化し、手動で作成した地図のみを参照する静的レイヤー（`static_layer`）のみで動作させます。
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "inflation_layer"] # obstacle_layerを除外

local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["static_layer", "inflation_layer"] # obstacle_layerを除外
```

---

## 4. マップ定義 (`map.png` / `map.yaml`) の作成仕様と計算方法

手動でCAD図面等からマップ画像を作成する際、**「画像のピクセル数」と「現実のサイズ（メートル）」を一致させる**ために、設定ファイル（`map.yaml`）の **`resolution`（解像度）** パラメータを使用します。

### ① 画像サイズ（ピクセル）と現実のサイズ（メートル）の計算式

画像のピクセル数は、以下の計算式で決定します。

$$\text{画像ピクセル数} = \frac{\text{現実のサイズ（メートル）}}{\text{解像度（resolution）}}$$

> [!NOTE]
> **解像度（resolution）** とは、**「1ピクセルが現実世界の何メートルに相当するか」** を示す値です。
> 通常は `0.05`（1ピクセル = 5cm）または `0.01`（1ピクセル = 1cm）を使用します。

#### 具体例：10m × 8m のフィールドのマップを作る場合
解像度を `0.05`（5cm/pixel）に設定するとします。
- **画像の横幅 (X)**: $10\text{ m} \div 0.05 = 200\text{ ピクセル}$
- **画像の縦幅 (Y)**: $8\text{ m} \div 0.05 = 160\text{ ピクセル}$

つまり、ペイントソフトで **「横 200ドット × 縦 160ドット」** の新規画像を作成すれば、現実の 10m × 8m と完全に一致します。

---

### ② 原点（`origin`）の決め方

YAMLファイルの `origin: [x, y, yaw]` は、**「作成した画像の左下角が、自己位置の基準点 (0, 0) から見てどこにあるか」** をメートル単位で示します。

```text
  ┌─────────────────────────────────┐ (右上)
  │                                 │
  │               ★(0, 0)           │
  │              スタート地点       │
  │                                 │
  ●─────────────────────────────────┘
(左下角 = origin)
```

- **例1：スタート地点 (0, 0) を画像の真ん中にしたい場合**（10m × 8m の画像の場合）
  左下角は `(0, 0)` から左に 5m、下に 4m の位置になるため、原点は以下のように設定します。
  `origin: [-5.0, -4.0, 0.0]`

- **例2：スタート地点 (0, 0) を画像の左下から「右に 1m、上に 1m」離れた位置にしたい場合**
  左下角は `(0, 0)` から左に 1m、下に 1m の位置になるため、原点は以下のように設定します。
  `origin: [-1.0, -1.0, 0.0]`

---

### ③ 画像ファイルのカラー仕様
- **白 (`255`)**: ロボットが通行可能なエリア
- **黒 (`0`)**: 壁や障害物、進入禁止エリア
- **灰色 (`205`付近)**: 未知領域。手動で自作する場合は、壁（黒）と床（白）の2色のみで作成し、灰色は使わなくても動作します。
- **画像フォーマット**: ロスレス圧縮で画質劣化がない `.png` または `.pgm` を推奨します。

### ④ YAMLファイルの記述例 (`map.yaml`)
```yaml
image: map.png
resolution: 0.05             # 1ピクセルあたり 0.05m (5cm)
origin: [-1.0, -1.0, 0.0]    # 画像の左下角のメートル座標
negate: 0
occupied_thresh: 0.65        # 黒とみなすしきい値
free_thresh: 0.25            # 白とみなすしきい値
```

