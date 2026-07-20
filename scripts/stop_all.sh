#!/bin/bash
# ============================================================
# honrobo_2026 全ノード一括停止スクリプト
# ============================================================

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 停止対象のセッションリスト
SESSIONS=("honrobo" "honrobo_robot" "honrobo_operator")

for SESSION_NAME in "${SESSIONS[@]}"; do
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        echo "  セッション '$SESSION_NAME' のノードを停止中..."
        for w in $(tmux list-windows -t "$SESSION_NAME" -F "#{window_index}" 2>/dev/null); do
            tmux send-keys -t "$SESSION_NAME:$w" C-c 2>/dev/null || true
        done
        sleep 1.5
        tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
        echo -e "${GREEN}  ✅ セッション '$SESSION_NAME' 終了${NC}"
    fi
done

# 残留プロセス（ラッパースクリプトおよびPythonノード）の強制終了
echo -e "${YELLOW}  残留プロセスを確認・クリーンアップ中...${NC}"

# 1. ラッパースクリプトを終了
pkill -f "run_node_wrapper.sh" 2>/dev/null || true

# 2. honrobo_pkg パッケージ関連の全プロセスを終了
pkill -f "honrobo_pkg" 2>/dev/null || true
sleep 0.5
pkill -9 -f "honrobo_pkg" 2>/dev/null || true

# 3. Web UI ポート(8080, 8765)を掴んでいるプロセスを強制解放
for port in 8080 8765; do
    if command -v fuser &> /dev/null; then
        fuser -k -n tcp $port &>/dev/null || true
    else
        # fuserが無い場合の代替 (lsof + kill)
        PID=$(lsof -t -i tcp:$port 2>/dev/null || true)
        if [ -n "$PID" ]; then
            kill -9 $PID 2>/dev/null || true
        fi
    fi
done

# 4. Nav2 関連の全プロセスを終了
pkill -f "nav2" 2>/dev/null || true
pkill -9 -f "nav2" 2>/dev/null || true

echo -e "${GREEN}  ✅ クリーンアップ完了${NC}"
