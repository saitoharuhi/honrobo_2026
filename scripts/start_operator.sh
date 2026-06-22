#!/bin/bash
# ============================================================
# honrobo_2026 操縦者側PC用 起動スクリプト (マルチPC構成用)
#
# 起動ノード: ps4_node
# ============================================================

set -e

SESSION_NAME="honrobo_operator"
WORKSPACE_DIR="$HOME/Documents/honrobo_2026"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m'

SKIP_BUILD=false
for arg in "$@"; do
    case $arg in
        --no-build) SKIP_BUILD=true ;;
        --help|-h)
            echo "使い方: bash scripts/start_operator.sh [--no-build]"
            exit 0 ;;
    esac
done

echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN} honrobo_2026 操縦者PC 起動 (マルチPC構成)${NC}"
echo -e "${CYAN}============================================${NC}"

# tmux チェック
if ! command -v tmux &> /dev/null; then
    echo -e "${RED}❌ tmux未インストール: sudo apt install tmux${NC}"
    exit 1
fi

# ビルド
if [ "$SKIP_BUILD" = false ]; then
    echo -e "${YELLOW}[1/2] ビルド中...${NC}"
    cd "$WORKSPACE_DIR"
    colcon build --symlink-install 2>&1 | tail -5
    echo -e "${GREEN}  ✅ ビルド完了${NC}"
else
    echo -e "${YELLOW}[1/2] ビルドスキップ${NC}"
fi

# tmux セッション準備
echo -e "${YELLOW}[2/2] 操縦者側ノード起動中...${NC}"
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

# ① ps4_node のみを起動
tmux new-session -d -s "$SESSION_NAME" -n "ps4"
tmux send-keys -t "$SESSION_NAME:ps4" "bash $WORKSPACE_DIR/scripts/run_node_wrapper.sh ps4_node" C-m

echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN} ✅ 操縦者側ノード (ps4_node) 起動完了!${NC}"
echo -e "${GREEN}============================================${NC}"
echo "  ※ロボットPC側で 'start_robot.sh' が起動していることを確認してください。"
echo ""
echo "  セッション接続:  tmux attach -t $SESSION_NAME"
echo "  停止:           bash scripts/stop_all.sh"
echo ""

tmux attach -t "$SESSION_NAME"
