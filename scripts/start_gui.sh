#!/bin/bash
# ============================================================
# 自律移動 監視・可視化GUI起動スクリプト (FastNav GUI)
# 
# 使い方: bash scripts/start_gui.sh [オプション]
# ============================================================

set -e

SCRIPTS_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPTS_DIR/.." && pwd)"

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
export ROS_LOCALHOST_ONLY=0

# ROS 2環境ソース
if [ -z "$ROS_DISTRO" ]; then
    for distro in humble foxy galactic iron jazzy; do
        if [ -d "/opt/ros/$distro" ]; then
            source "/opt/ros/$distro/setup.bash"
            break
        fi
    done
fi

if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi

echo "============================================"
echo " 🚀 FastNav 自律移動 監視GUI を起動中..."
echo "============================================"
echo " ・左クリック: 目標地点(Goal)送信"
echo " ・スペースキー: 緊急停止 (STOP)"
echo " ・マウスホイール: 拡大/縮小"
echo " ・右ドラッグ: マップ移動 (Pan)"
echo " ・Rキー: 画面リセット / Fキー: ロボット追従"
echo "============================================"

ros2 run honrobo_pkg nav_gui "$@"
