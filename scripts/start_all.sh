#!/bin/bash
# ============================================================
# honrobo_2026 全ノード一括起動スクリプト
# tmuxを使用して各ノードを個別のウィンドウで起動します
#
# 使い方: bash scripts/start_all.sh [オプション]
# オプション:
#   --no-can     CAN初期設定をスキップ
#   --no-build   ビルドをスキップ
# ============================================================

set -e

SESSION_NAME="honrobo"
SCRIPTS_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPTS_DIR/.." && pwd)"

# カラー
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m'

# オプション解析
SKIP_CAN=false
SKIP_BUILD=false
for arg in "$@"; do
    case $arg in
        --no-can)   SKIP_CAN=true ;;
        --no-build) SKIP_BUILD=true ;;
        --help|-h)
            echo "使い方: bash scripts/start_all.sh [--no-can] [--no-build]"
            exit 0 ;;
    esac
done

echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN} honrobo_2026 一括起動${NC}"
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

# ビルド
if [ "$SKIP_BUILD" = false ]; then
    echo -e "${YELLOW}[2/3] ビルド中...${NC}"
    cd "$WORKSPACE_DIR"
    colcon build --symlink-install 2>&1 | tail -5
    echo -e "${GREEN}  ✅ ビルド完了${NC}"
else
    echo -e "${YELLOW}[2/3] ビルドスキップ${NC}"
fi

# tmux セッション準備
echo -e "${YELLOW}[3/3] ノード起動中...${NC}"
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

SETUP_CMD="source /opt/ros/\$(ls /opt/ros/ | head -1)/setup.bash && source $WORKSPACE_DIR/install/setup.bash"

# ① zikoiti_node (自己位置推定)
tmux new-session -d -s "$SESSION_NAME" -n "sensor"
tmux send-keys -t "$SESSION_NAME:sensor" "bash $WORKSPACE_DIR/scripts/run_node_wrapper.sh zikoiti_node" C-m
sleep 1

# ② can_node (CAN通信)
tmux new-window -t "$SESSION_NAME" -n "can"
tmux send-keys -t "$SESSION_NAME:can" "bash $WORKSPACE_DIR/scripts/run_node_wrapper.sh can_node" C-m
sleep 0.5

# ③ ps4_node (PS4コントローラー)
tmux new-window -t "$SESSION_NAME" -n "ps4"
tmux send-keys -t "$SESSION_NAME:ps4" "bash $WORKSPACE_DIR/scripts/run_node_wrapper.sh ps4_node" C-m
sleep 0.5

# ④ roboware_node (制御統合)
tmux new-window -t "$SESSION_NAME" -n "roboware"
tmux send-keys -t "$SESSION_NAME:roboware" "bash $WORKSPACE_DIR/scripts/run_node_wrapper.sh roboware_node" C-m
sleep 0.5

# ⑤ web_node (WebSocket + HTTP)
tmux new-window -t "$SESSION_NAME" -n "web"
tmux send-keys -t "$SESSION_NAME:web" "bash $WORKSPACE_DIR/scripts/run_node_wrapper.sh web_node" C-m

tmux select-window -t "$SESSION_NAME:sensor"

echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN} ✅ 全ノード起動完了!${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo "  セッション接続:  tmux attach -t $SESSION_NAME"
echo "  ウィンドウ切替:  Ctrl+B → 数字(0-4)"
echo "  セッション離脱:  Ctrl+B → d"
echo "  停止:           bash scripts/stop_all.sh"
echo ""
echo "  [0] sensor   - zikoiti_node (自己位置推定)"
echo "  [1] can      - can_node (CAN通信)"
echo "  [2] ps4      - ps4_node (PS4コントローラー)"
echo "  [3] roboware - roboware_node (制御統合)"
echo "  [4] web      - web_node (WebSocket/HTTP)"
echo ""

tmux attach -t "$SESSION_NAME"
