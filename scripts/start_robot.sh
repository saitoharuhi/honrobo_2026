#!/bin/bash
# ============================================================
# honrobo_2026 ロボット側PC用 起動スクリプト (マルチPC構成用)
#
# 起動ノード: zikoiti_node, can_node, roboware_node, web_node
# ============================================================

set -e

# ============================================================
# マルチPC通信用設定 (デフォルトでROS_DOMAIN_ID=30、LOCALHOST_ONLY=0を適用)
# ============================================================
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
export ROS_LOCALHOST_ONLY=0

SESSION_NAME="honrobo_robot"
SCRIPTS_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPTS_DIR/.." && pwd)"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m'

SKIP_CAN=false
SKIP_BUILD=false
for arg in "$@"; do
    case $arg in
        --no-can)   SKIP_CAN=true ;;
        --no-build) SKIP_BUILD=true ;;
        --help|-h)
            echo "使い方: bash scripts/start_robot.sh [--no-can] [--no-build]"
            exit 0 ;;
    esac
done

echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN} honrobo_2026 ロボットPC 起動 (マルチPC構成)${NC}"
echo -e "${CYAN}============================================${NC}"

# tmux チェック
if ! command -v tmux &> /dev/null; then
    echo -e "${RED}❌ tmux未インストール: sudo apt install tmux${NC}"
    exit 1
fi

# CAN セットアップ
if [ "$SKIP_CAN" = false ]; then
    echo -e "${YELLOW}[1/3] CAN通信セットアップ${NC}"
    sudo bash "$SCRIPTS_DIR/setup_can.sh"
else
    echo -e "${YELLOW}[1/3] CANスキップ${NC}"
fi

# シリアルポートの権限自動付与
echo -e "${YELLOW}[1.5/3] シリアルポートの権限付与中...${NC}"
sudo chmod 666 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true

# 依存Pythonライブラリのチェック
echo -e "${YELLOW}[1.8/3] Python依存ライブラリをチェック中...${NC}"
MISSING_LIBS=()
python3 -c "import pygame" 2>/dev/null || MISSING_LIBS+=("pygame")
python3 -c "import can" 2>/dev/null || MISSING_LIBS+=("python-can")
python3 -c "import websockets" 2>/dev/null || MISSING_LIBS+=("websockets")

if [ ${#MISSING_LIBS[@]} -ne 0 ]; then
    echo -e "${RED}❌ 以下の必須Pythonライブラリがインストールされていません:${NC}"
    for lib in "${MISSING_LIBS[@]}"; do
        echo "  - $lib"
    done
    echo ""
    echo "  以下のコマンドを実行してインストールしてください:"
    echo "  pip3 install pygame python-can websockets"
    echo "  (または: sudo apt install python3-pygame python3-websockets && pip3 install python-can)"
    exit 1
fi
echo -e "${GREEN}  ✅ 依存ライブラリOK${NC}"

# ROS 2環境の自動ソース & ビルド
if [ "$SKIP_BUILD" = false ]; then
    echo -e "${YELLOW}[2/3] ビルド準備中 (ROS 2環境の確認)${NC}"
    if [ -z "$ROS_DISTRO" ]; then
        ROS_PATH=""
        for distro in humble foxy galactic iron jazzy; do
            if [ -d "/opt/ros/$distro" ]; then
                ROS_PATH="/opt/ros/$distro/setup.bash"
                break
            fi
        done
        if [ -n "$ROS_PATH" ]; then
            echo -e "${GREEN}  -> ${ROS_PATH} をロードします${NC}"
            source "$ROS_PATH"
        else
            ANY_ROS=$(ls /opt/ros/ 2>/dev/null | head -1)
            if [ -n "$ANY_ROS" ]; then
                echo -e "${GREEN}  -> /opt/ros/$ANY_ROS/setup.bash をロードします${NC}"
                source "/opt/ros/$ANY_ROS/setup.bash"
            else
                echo -e "${RED}❌ ROS 2環境が見つかりません。ROS 2をインストールするか、sourceしてください。${NC}"
                exit 1
            fi
        fi
    fi

    echo -e "${YELLOW}  ビルド中...${NC}"
    cd "$WORKSPACE_DIR"
    
    # 他PCからのコピー等によるシンボリックリンク破損や絶対パスの不整合を考慮したビルド
    BUILD_SUCCESS=true
    # 一旦警告無視でビルドを試みる
    PYTHONWARNINGS=ignore colcon build --symlink-install > /tmp/colcon_build.log 2>&1 || BUILD_SUCCESS=false
    
    if [ "$BUILD_SUCCESS" = false ]; then
        echo -e "${YELLOW}⚠️ ビルドが失敗しました。古いビルドキャッシュをクリーンアップして再試行します...${NC}"
        # 古いディレクトリを完全削除
        rm -rf build/ install/ log/ 2>/dev/null || true
        # 再度ビルドを実行
        echo -e "${YELLOW}  再ビルド中...${NC}"
        PYTHONWARNINGS=ignore colcon build --symlink-install 2>&1 | tail -5 || {
            echo -e "${RED}❌ 再ビルドも失敗しました。ログを確認してください:${NC}"
            cat /tmp/colcon_build.log | tail -20
            exit 1
        }
    else
        # 成功時は最後の数行だけ表示
        cat /tmp/colcon_build.log | tail -5
    fi
    source "$WORKSPACE_DIR/install/setup.bash"
    echo -e "${GREEN}  ✅ ビルド完了${NC}"
else
    echo -e "${YELLOW}[2/3] ビルドスキップ${NC}"
    if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        source "$WORKSPACE_DIR/install/setup.bash"
    fi
fi

# 自己位置ソースの確認
echo -e "\n${CYAN}============================================"
echo -e " 自己位置ソース (Odometry Source) の選択"
echo -e "============================================${NC}"
echo "1) 外部マイコンを使用する (Microcontroller via Serial)"
echo "2) PC側ノードを使用する (zikoiti_node / OTOS + Gyro via Arduino)"
read -p "選択 [1-2] (デフォルト: 1): " ODOM_INPUT

USE_MICRO=true
if [ "$ODOM_INPUT" = "2" ]; then
    echo -e "${GREEN}  -> PC側自己位置ノード (zikoiti_node / OTOS + Gyro via Arduino) を使用します。${NC}"
    USE_MICRO=false
else
    echo -e "${GREEN}  -> 外部マイコン自己位置 (Serial直接受信) を使用します。${NC}"
    USE_MICRO=true
fi
echo ""

# 競技エリア・マップ (Map Zone) の選択
echo -e "\n${CYAN}============================================"
echo -e " 競技エリア・マップ (Map Zone) の選択"
echo -e "============================================${NC}"
echo "1) 赤ゾーン (Red Zone - Left Side)"
echo "2) 青ゾーン (Blue Zone - Right Side)"
read -p "選択 [1-2] (デフォルト: 1): " ZONE_INPUT

MAP_FILE="map_red.yaml"
if [ "$ZONE_INPUT" = "2" ]; then
    echo -e "${GREEN}  -> 青ゾーン用のマップ (map_blue.yaml) を使用します。${NC}"
    MAP_FILE="map_blue.yaml"
else
    echo -e "${GREEN}  -> 赤ゾーン用のマップ (map_red.yaml) を使用します。${NC}"
    MAP_FILE="map_red.yaml"
fi
echo ""

# tmux セッション準備
echo -e "${YELLOW}[3/3] ロボット側ノード起動中...${NC}"
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

SETUP_CMD="source /opt/ros/\$(ls /opt/ros/ | head -1)/setup.bash && source $WORKSPACE_DIR/install/setup.bash"

# ① zikoiti_node (自己位置推定 / 外部マイコンシリアル受信)
tmux new-session -d -s "$SESSION_NAME" -n "sensor"
if [ "$USE_MICRO" = true ]; then
    tmux send-keys -t "$SESSION_NAME:sensor" "bash $WORKSPACE_DIR/scripts/run_node_wrapper.sh zikoiti_node --use-micro" C-m
else
    tmux send-keys -t "$SESSION_NAME:sensor" "bash $WORKSPACE_DIR/scripts/run_node_wrapper.sh zikoiti_node" C-m
fi
sleep 1

# ② can_node
tmux new-window -t "$SESSION_NAME" -n "can"
tmux send-keys -t "$SESSION_NAME:can" "bash $WORKSPACE_DIR/scripts/run_node_wrapper.sh can_node" C-m
sleep 0.5

# ③ roboware_node
tmux new-window -t "$SESSION_NAME" -n "roboware"
tmux send-keys -t "$SESSION_NAME:roboware" "bash $WORKSPACE_DIR/scripts/run_node_wrapper.sh roboware_node" C-m
sleep 0.5

# ④ web_node
tmux new-window -t "$SESSION_NAME" -n "web"
tmux send-keys -t "$SESSION_NAME:web" "bash $WORKSPACE_DIR/scripts/run_node_wrapper.sh web_node" C-m
sleep 0.5

# ⑤ nav2 (Nav2自律移動スタック)
tmux new-window -t "$SESSION_NAME" -n "nav2"
tmux send-keys -t "$SESSION_NAME:nav2" "$SETUP_CMD && ros2 launch honrobo_pkg nav2.launch.py map:=\$(ros2 pkg prefix honrobo_pkg)/share/honrobo_pkg/map/$MAP_FILE" C-m

tmux select-window -t "$SESSION_NAME:sensor"

echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN} ✅ ロボット側ノード起動完了!${NC}"
echo -e "${GREEN}============================================${NC}"
echo "  ※操縦者PC側で 'start_operator.sh' または 'operator.sh' を起動してください。"
echo ""
echo "  セッション接続:  tmux attach -t $SESSION_NAME"
echo "  ウィンドウ切替:  Ctrl+B → 数字(0-4)"
echo "  停止:           bash scripts/stop_all.sh"
echo ""
echo "  [0] sensor   - zikoiti_node (自己位置推定)"
echo "  [1] can      - can_node (CAN通信)"
echo "  [2] roboware - roboware_node (制御統合)"
echo "  [3] web      - web_node (WebSocket/HTTP)"
echo "  [4] nav2     - nav2.launch.py (Nav2自律移動スタック)"
echo ""

tmux attach -t "$SESSION_NAME"
