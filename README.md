# honrobo_pkg — 2026年ロボット制御パッケージ

ROS 2 ベースのロボット制御システムです。  
オドメトリセンサーとジャイロによる自己位置推定、または外部マイコンから送られてくる自己位置値を使用し、PS4コントローラーによるフィールド基準手動操作、WebSocketを介した自動運転制御、CAN通信によるマイコン制御を統合します。

## アーキテクチャ

起動時の選択により、自己位置（`/odom`）の配信元を「外部マイコン（シリアル経由）」と「PC側ノード（IMU+OTOS）」で切り替え可能です。手動操縦時には、ロボットの角度 `Yaw` に応じて自動的に入力が回転変換（フィールド基準操縦）されます。

```text
[IMU/OTOS] ────┐
               ├─(シリアル)─→ zikoiti_node ──→ /odom ─┬─→ roboware_node (※フィールド基準回転変換) ──→ /can_tx ──→ can_node ──→ [マイコン]
[マイコン] ────┘                                  │         ↑
                                                   │   /ps4_joy│
[PS4 Controller] ───────────────→ ps4_node ──┘         │
                                                       /nav_cmd, /auto_mode
[ブラウザ] ←─────────────────────────→ web_node ─────────────────────────────┘
```

## ノード一覧

| ノード | コマンド | 役割 |
|--------|----------|------|
| `zikoiti_node` | `ros2 run honrobo_pkg zikoiti_node` | 自己位置推定＆配信 (通常: IMU+OTOS統合 / 外部マイコン時: シリアルから直接受信して配信) |
| `can_node` | `ros2 run honrobo_pkg can_node` | CAN通信 (操作指令などの送信) |
| `ps4_node` | `ros2 run honrobo_pkg ps4_node` | PS4コントローラー入力 (ヘッドレス環境対応) |
| `roboware_node` | `ros2 run honrobo_pkg roboware_node` | 手動/自動制御の統合 (フィールド基準座標変換を含む) |
| `web_node` | `ros2 run honrobo_pkg web_node` | WebSocket + Web UI |

## セットアップ

### 1. ビルド
```bash
cd ~/Documents/honrobo_2026
colcon build --symlink-install
source install/setup.bash
```

### 2. CAN通信の設定
```bash
sudo bash scripts/setup_can.sh
```

### 3. 一括起動
```bash
bash scripts/start_all.sh
```
※ロボット側PCと操縦者側PCに分けて起動する場合は、それぞれ `start_robot.sh` / `start_operator.sh` を使用します。

#### 💡 自己位置ソースの選択プロンプト
起動スクリプトを実行すると、ターミナルで自己位置（オドメトリ）のソース選択を求められます。
*   **1) 外部マイコンを使用する (デフォルト: 1)**
    PC側の `zikoiti_node` は起動せず、マイコンから `0x520` IDでCAN送信されてくる自己位置を受信して使用します。
*   **2) PC側ノードを使用する**
    PCに接続されたジャイロ（IMU）およびOTOSセンサーから `zikoiti_node` がデータを読み取って自己位置を生成します。

#### 起動スクリプトのオプション
| オプション | 説明 |
|-----------|------|
| `--no-can` | CAN初期化処理をスキップ |
| `--no-build` | 起動前の colcon build をスキップ |

### tmux 操作方法

| 操作 | キー |
|------|------|
| ウィンドウ切替 | `Ctrl+B` → 数字 (0-4) |
| 次のウィンドウ | `Ctrl+B` → `n` |
| セッション離脱 | `Ctrl+B` → `d` |
| 再接続 (一括起動) | `tmux attach -t honrobo` |
| 再接続 (ロボット単体) | `tmux attach -t honrobo_robot` |

### 4. 一括停止
```bash
bash scripts/stop_all.sh
```

## フィールド基準手動操縦
`roboware_node` 内に、自己位置の `Yaw` 角度に基づく回転座標変換が組み込まれています。
これにより、**ロボットがフィールド上でどの向きを向いていても、操縦者がコントローラーのスティックを「上」に倒せば、ロボットは常に操縦者から見て「まっすぐ前（フィールドの奥）」へ進みます。**

## Web UIの使い方

1. 全ノードを起動します（`web_node` も含みます）。
2. ブラウザで `http://<ロボットPCのIP>:8080` にアクセスします。
3. WebSocket経由で自己位置データや制御状態がリアルタイムでブラウザに送信されます。
4. 画面上のマップ上のプリセットボタンを押すことで、自動運転（WebSocket通信経由）を開始できます。
5. 緊急停止は、画面上の「EMERGENCY STOP」ボタンを押すか、**PS4コントローラーのいずれかのボタン（形状キー、矢印キー、L/Rボタン等）を押すか、またはスティックを任意の方向へ少しでも動かす**ことで実行できます。コントローラー操作を検知した瞬間、自動運転は即座にキャンセルされ、ロボットは速度0で非常停止して手動操縦モードに復帰します。

*   **ロボットPCのIPアドレス確認方法**: `hostname -I` で確認可能です。

## CAN通信プロトコル

| CAN ID | 送受信 | データ | 説明 |
|--------|--------|--------|------|
| `0x510` | 送信(TX) | VX(2B) + VY(2B) + Vω(2B) | 速度指令 (int16 BE, mm/s) |
| `0x500` | 送信(TX) | ○△×□↑↓←→ | 形状+矢印ボタン (各1B) |
| `0x501` | 送信(TX) | R1,R2,R3,L1,L2,L3 | L/Rボタン (各1B) |
| `0x502` | 送信(TX) | Share, Options, PS | システムボタン (各1B) |
## ファイル構成
```
Documents/honrobo_2026/
├── scripts/
│   ├── setup_can.sh        # CAN初期設定
│   ├── start_all.sh        # 全ノード一括起動 (対話プロンプト付き)
│   ├── start_robot.sh      # ロボット側一括起動 (対話プロンプト付き)
│   ├── start_operator.sh   # 操縦者側一括起動
│   └── stop_all.sh         # 全ノード一括停止
└── src/honrobo_pkg/
    ├── honrobo_pkg/
    │   ├── zikoiti_node.py  # 自己位置推定＆受信 (OTOS+Gyro / 外部マイコンシリアル受信)
    │   ├── can_node.py      # CAN送受信 (操作指令などの送信)
    │   ├── ps4_node.py      # PS4入力 (ヘッドレス環境対応)
    │   ├── roboware_node.py # 制御統合 (フィールド基準座標変換)
    │   └── web_node.py      # WebSocket/HTTP WebUI
    ├── setup.py
    └── package.xml
```

