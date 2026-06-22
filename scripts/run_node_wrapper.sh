#!/bin/bash
# ============================================================
# honrobo_2026 ノード実行ラッパースクリプト
# 
# 役割: 
# 1. 各ROS2ノードを起動し、その生存を監視します。
# 2. Ctrl+C 等でノードが終了した際、対話プロンプトを表示して
#    「個別停止したままにする」「再起動する」「すべて停止する」を選択させます。
#
# 使い方: run_node_wrapper.sh <node_name>
# ============================================================

NODE_NAME=$1
WORKSPACE_DIR="/home/haru/Documents/honrobo_2026"
SETUP_CMD="source /opt/ros/\$(ls /opt/ros/ | head -1)/setup.bash && source $WORKSPACE_DIR/install/setup.bash"

# 環境の読み込み
eval "$SETUP_CMD"

while true; do
    echo -e "\n=== Starting $NODE_NAME ==="
    
    # ROS 2 ノードを実行
    ros2 run honrobo_pkg "$NODE_NAME"
    EXIT_CODE=$?
    
    # ターミナルの入力バッファをクリア（実行中に押された不要なキー入力を捨てる）
    stty sane 2>/dev/null
    
    echo -e "\n=============================================="
    echo -e " 🔴 プログラム '$NODE_NAME' が終了しました。 (コード: $EXIT_CODE)"
    echo -e "=============================================="
    echo -e "  [1] このウィンドウ ($NODE_NAME) のみ停止したままにする"
    echo -e "  [2] このノード ($NODE_NAME) を再起動する"
    echo -e "  [3] 全てのプログラムを終了する (stop_all.shを実行)"
    echo -e "=============================================="
    
    read -p " 選択してください [1-3] (デフォルト: 1): " choice
    
    if [ -z "$choice" ]; then
        choice=1
    fi
    
    case "$choice" in
        2)
            echo "再起動中..."
            sleep 1
            clear
            continue
            ;;
        3)
            echo "すべてのノードを終了します..."
            bash "$WORKSPACE_DIR/scripts/stop_all.sh"
            exit 0
            ;;
        1|*)
            echo "このウィンドウのノードを停止しました。"
            echo "このウィンドウ（ペイン）を閉じるには、Ctrl+B -> x を押すか、exit を実行してください。"
            # 無限待機して、ユーザーがウィンドウを手動で閉じるまで生存させる
            while true; do sleep 60; done
            exit 0
            ;;
    esac
done
