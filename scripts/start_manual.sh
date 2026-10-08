#!/bin/bash
# 手動運転専用の起動スクリプト (Nav2やLiDARを起動しない軽量版)

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPTS_DIR="$WORKSPACE_DIR/scripts"
SESSION_NAME="honrobo_manual"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}============================================"
echo -e " 🚀 2026 ほんロボ 手動運転専用モード起動"
echo -e "============================================${NC}"

# 引数解析
SKIP_BUILD=false
SKIP_CAN=false
for arg in "$@"; do
    if [ "$arg" = "--skip-build" ]; then SKIP_BUILD=true; fi
    if [ "$arg" = "--no-can" ]; then SKIP_CAN=true; fi
done

# ==========================================
# システム初期化 (権限・CAN等)
# ==========================================

# ROS 2 環境変数の設定 (すでにsourceされていればスキップ)
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}[0/3] ROS 2 環境をセットアップ中...${NC}"
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    else
        ROS_SETUP=$(ls /opt/ros/*/setup.bash 2>/dev/null | head -1)
        if [ -n "$ROS_SETUP" ]; then
            source "$ROS_SETUP"
        else
            echo -e "${RED}❌ ROS 2環境が見つかりません。ROS 2をインストールするか、sourceしてください。${NC}"
            exit 1
        fi
    fi
fi

# CAN セットアップ (CANableの接続有無を自動判定)
if [ "$SKIP_CAN" = false ]; then
    if ip link show can0 2>/dev/null | grep -q "UP"; then
        echo -e "${GREEN}[1/3] ✅ can0 は既にアクティブ(UP)です。実CAN通信で起動します。${NC}"
        SKIP_CAN=false
    else
        CANABLE_DETECTED=false
        if ip link show can0 &>/dev/null; then
            CANABLE_DETECTED=true
        else
            CANABLE_CHECK=$(python3 -c "
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
found = any('canable' in p.description.lower() or '16d0:117e' in p.hwid.lower() for p in ports)
print('true' if found else 'false')
" 2>/dev/null || echo "false")
            if [ "$CANABLE_CHECK" = "true" ]; then
                CANABLE_DETECTED=true
            fi
        fi

        if [ "$CANABLE_DETECTED" = true ]; then
            echo -e "${YELLOW}[1/3] CAN通信セットアップ中...${NC}"
            if sudo bash "$SCRIPTS_DIR/setup_can.sh"; then
                echo -e "${GREEN}  ✅ CANセットアップ完了${NC}"
                SKIP_CAN=false
            else
                echo -e "${YELLOW}  ⚠️ CANセットアップに失敗したため、モックモード (--no-can) にフォールバックします。${NC}"
                SKIP_CAN=true
            fi
        else
            echo -e "${YELLOW}[1/3] ⚠️ CANableが未検出です。CAN通信をモックモード (--no-can) で自動起動します。${NC}"
            SKIP_CAN=true
        fi
    fi
else
    echo -e "${YELLOW}[1/3] CANスキップ (--no-can 指定)${NC}"
fi

# シリアルポートの権限自動付与
echo -e "${YELLOW}[1.5/3] シリアルポートの権限付与中...${NC}"
sudo chmod 666 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true

# ==========================================

# ビルド
if [ "$SKIP_BUILD" = false ]; then
    echo -e "${YELLOW}[2/3] ビルド中...${NC}"
    cd "$WORKSPACE_DIR"
    PYTHONWARNINGS=ignore colcon build --symlink-install > /tmp/colcon_build.log 2>&1
    source "$WORKSPACE_DIR/install/setup.bash"
else
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
    USE_MICRO=false
fi

# tmux セッション準備
echo -e "${YELLOW}[3/3] ノード起動中...${NC}"
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

# ① zikoiti_node (自己位置推定 / 外部マイコンシリアル受信)
tmux new-session -d -s "$SESSION_NAME" -n "sensor"
if [ "$USE_MICRO" = true ]; then
    tmux send-keys -t "$SESSION_NAME:sensor" "bash $WORKSPACE_DIR/scripts/run_node_wrapper.sh zikoiti_node --use-micro" C-m
else
    tmux send-keys -t "$SESSION_NAME:sensor" "bash $WORKSPACE_DIR/scripts/run_node_wrapper.sh zikoiti_node" C-m
fi
sleep 1

# ② can_node (CAN通信)
tmux new-window -t "$SESSION_NAME" -n "can"
if [ "$SKIP_CAN" = true ]; then
    tmux send-keys -t "$SESSION_NAME:can" "bash $WORKSPACE_DIR/scripts/run_node_wrapper.sh can_node --no-can" C-m
else
    tmux send-keys -t "$SESSION_NAME:can" "bash $WORKSPACE_DIR/scripts/run_node_wrapper.sh can_node" C-m
fi
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
sleep 0.5

# ⑥ Wi-Fi Watchdog (自動復旧)
tmux new-window -t "$SESSION_NAME" -n "wifi_check"
tmux send-keys -t "$SESSION_NAME:wifi_check" "bash $WORKSPACE_DIR/scripts/wifi_watchdog.sh" C-m
sleep 0.5

tmux select-window -t "$SESSION_NAME:sensor"

echo ""
echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN} ✅ 手動運転専用モード起動完了!${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo "  セッション接続:  tmux attach -t $SESSION_NAME"
echo "  ウィンドウ切替:  Ctrl+B → 数字(0-5)"
echo "  セッション離脱:  Ctrl+B → d"
echo "  停止:           bash scripts/stop_all.sh"
echo ""
echo "  [0] sensor     - zikoiti_node (自己位置推定)"
echo "  [1] can        - can_node (CAN通信)"
echo "  [2] ps4        - ps4_node (PS4コントローラー)"
echo "  [3] roboware   - roboware_node (制御統合)"
echo "  [4] web        - web_node (WebSocket/HTTP)"
echo "  [5] wifi_check - Wi-Fi自動復旧監視"
echo ""

tmux attach -t "$SESSION_NAME"
