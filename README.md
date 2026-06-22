# honrobo_pkg — 2026年ロボット制御パッケージ

ROS 2 ベースのロボット制御システムです。  
オドメトリセンサーとジャイロによる自己位置推定、PS4コントローラーによる手動操作、WebSocketを介した自動運転制御、CAN通信によるマイコン制御を統合します。

## アーキテクチャ

```
[IMU ジャイロ] ─┐
                ├─→ zikoiti_node ──→ /odom ─┬─→ roboware_node ──→ /can_tx ──→ can_node ──→ [マイコン]
[Arduino OTOS] ─┘                           │         ↑
                                            │   /ps4_joy│
[PS4 Controller] ──→ ps4_node ──────────────┘         │
                                                 /nav_cmd, /auto_mode
[ブラウザ] ←──→ web_node ─────────────────────────────┘
```

## ノード一覧

| ノード | コマンド | 役割 |
|--------|----------|------|
| `zikoiti_node` | `ros2 run honrobo_pkg zikoiti_node` | 自己位置推定 (IMU+OTOS統合) |
| `can_node` | `ros2 run honrobo_pkg can_node` | CAN通信 (x,y,ω を16進数送信) |
| `ps4_node` | `ros2 run honrobo_pkg ps4_node` | PS4コントローラー入力 |
| `roboware_node` | `ros2 run honrobo_pkg roboware_node` | 手動/自動制御の統合 |
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

### 3. 一括起動 (推奨)
```bash
bash scripts/start_all.sh
```

全ノードが `tmux` の個別ウィンドウで起動します。

| オプション | 説明 |
|-----------|------|
| `--no-can` | CAN設定をスキップ |
| `--no-build` | ビルドをスキップ |

### tmux 操作方法

| 操作 | キー |
|------|------|
| ウィンドウ切替 | `Ctrl+B` → 数字 (0-4) |
| 次のウィンドウ | `Ctrl+B` → `n` |
| セッション離脱 | `Ctrl+B` → `d` |
| 再接続 | `tmux attach -t honrobo` |

### 4. 一括停止
```bash
bash scripts/stop_all.sh
```

## Web UIの使い方

1. 全ノードを起動
2. ブラウザで `http://<ロボットPCのIP>:8080` にアクセス
3. マップ上のボタンを押して自動運転開始
4. 緊急停止: EMERGENCY STOP ボタン or PS4操作

IPアドレス確認: `hostname -I`

## CAN通信プロトコル

| CAN ID | データ | 説明 |
|--------|--------|------|
| `0x510` | VX(2B) + VY(2B) + Vω(2B) | 速度指令 (int16 BE, mm/s) |
| `0x500` | ○△×□↑↓←→ | 形状+矢印ボタン (各1B) |
| `0x501` | R1,R2,R3,L1,L2,L3 | L/Rボタン (各1B) |
| `0x502` | Share, Options, PS | システムボタン (各1B) |

## ファイル構成
```
Documents/honrobo_2026/
├── scripts/
│   ├── setup_can.sh        # CAN初期設定
│   ├── start_all.sh        # 全ノード一括起動
│   └── stop_all.sh         # 全ノード一括停止
└── src/honrobo_pkg/
    ├── honrobo_pkg/
    │   ├── zikoiti_node.py  # 自己位置推定
    │   ├── can_node.py      # CAN通信
    │   ├── ps4_node.py      # PS4入力
    │   ├── roboware_node.py # 制御統合
    │   └── web_node.py      # WebSocket/HTTP
    ├── setup.py
    └── package.xml
```
