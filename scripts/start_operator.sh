#!/bin/bash
# ============================================================
# honrobo_2026 操縦者側PC用 起動スクリプト (マルチPC構成用)
#
# 起動ノード: ps4_node
# ============================================================

set -e

# ============================================================
# マルチPC通信用設定 (デフォルトでROS_DOMAIN_ID=30、LOCALHOST_ONLY=0を適用)
# ============================================================
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
export ROS_LOCALHOST_ONLY=0

SESSION_NAME="honrobo_operator"
SCRIPTS_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPTS_DIR/.." && pwd)"

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

# 依存Pythonライブラリのチェック
echo -e "${YELLOW}Python依存ライブラリをチェック中...${NC}"
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
    echo -e "${YELLOW}[1/2] ビルド準備中 (ROS 2環境の確認)${NC}"
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
    echo -e "${YELLOW}[1/2] ビルドスキップ${NC}"
    if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        source "$WORKSPACE_DIR/install/setup.bash"
    fi
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
