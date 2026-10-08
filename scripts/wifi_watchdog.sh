#!/bin/bash
# Wi-Fi 自動復旧＆ROS2自動再起動スクリプト (強化版)

echo "=========================================="
echo " 📡 Wi-Fi 監視機能を起動しました"
echo " (Wi-Fi子機の抜き差しを監視します)"
echo "=========================================="

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 前回のWi-Fiインターフェース一覧（デバイス名のみ）
PREV_WIFI_DEVS=""
# 再接続待ちフラグ
WAITING_FOR_RECONNECT=false

while true; do
    # 現在のWi-Fiデバイス一覧を取得 (例: wlan0 wlan1)
    CUR_WIFI_DEVS=$(nmcli -t -f DEVICE,TYPE dev | grep "wifi" | cut -d':' -f1 | sort | tr '\n' ' ')

    # 起動時の初期化
    if [ -z "$PREV_WIFI_DEVS" ] && [ -n "$CUR_WIFI_DEVS" ]; then
        PREV_WIFI_DEVS="$CUR_WIFI_DEVS"
        echo "$(date '+%H:%M:%S') ℹ️ 現在認識されているWi-Fi: $CUR_WIFI_DEVS"
    fi

    # デバイス一覧に変化があったかチェック（抜かれた or 刺された）
    if [ "$CUR_WIFI_DEVS" != "$PREV_WIFI_DEVS" ]; then
        echo "$(date '+%H:%M:%S') ⚠️ Wi-Fiデバイスの構成が変化しました！"
        echo "  [前回] $PREV_WIFI_DEVS"
        echo "  [現在] $CUR_WIFI_DEVS"

        # デバイスが減った場合（抜かれた）
        if [ ${#CUR_WIFI_DEVS} -lt ${#PREV_WIFI_DEVS} ]; then
            echo "$(date '+%H:%M:%S') 🔌 USB Wi-Fi子機が抜かれました。再挿入を待機します..."
            WAITING_FOR_RECONNECT=true
        fi

        # デバイスが増えた場合（刺された）
        if [ ${#CUR_WIFI_DEVS} -gt ${#PREV_WIFI_DEVS} ]; then
            echo "$(date '+%H:%M:%S') 🔌 USB Wi-Fi子機が刺されました！"
            echo "ネットワークへの自動接続を待機中..."
            WAITING_FOR_RECONNECT=true
            
            # nmcliで再スキャンを促す
            nmcli device wifi rescan 2>/dev/null || true
        fi
        
        PREV_WIFI_DEVS="$CUR_WIFI_DEVS"
    fi

    # 再接続待ち状態の場合、どれか1つでも「connected」になれば復旧とみなす
    if [ "$WAITING_FOR_RECONNECT" = true ]; then
        # 接続済みのWi-Fiデバイスがあるかチェック
        ANY_CONNECTED=$(nmcli -t -f DEVICE,TYPE,STATE dev | grep "wifi:connected" || true)
        
        if [ -n "$ANY_CONNECTED" ]; then
            echo "$(date '+%H:%M:%S') ✅ Wi-Fiネットワークに再接続されました！"
            echo "ROS 2 の stale 状態を解消するため、ノードを自動再起動します..."
            
            # 3秒待つ（IPアドレス割り当てなどを確実にするため）
            sleep 3
            
            # 現在のセッション名を取得
            SESSION_NAME=""
            if tmux ls 2>/dev/null | grep -q "honrobo_operator"; then
                SESSION_NAME="honrobo_operator"
            elif tmux ls 2>/dev/null | grep -q "honrobo_manual"; then
                SESSION_NAME="honrobo_manual"
            elif tmux ls 2>/dev/null | grep -q "honrobo"; then
                SESSION_NAME="honrobo"
            fi

            if [ -n "$SESSION_NAME" ]; then
                echo "🚀 セッション $SESSION_NAME 内のROS 2ノードを再起動中..."
                # tmuxの全ペインを列挙し、wifi_check以外にCtrl+C -> 2 (再起動) を送信
                for p in $(tmux list-panes -a -F "#{session_name}:#{window_name}:#{pane_id}" 2>/dev/null | grep "^$SESSION_NAME:"); do
                    W_NAME=$(echo "$p" | cut -d':' -f2)
                    P_ID=$(echo "$p" | cut -d':' -f3)
                    if [ "$W_NAME" != "wifi_check" ]; then
                        # 終了シグナル送信
                        tmux send-keys -t "$P_ID" C-c
                        sleep 0.5
                        # run_node_wrapper.sh の再起動選択肢 '2' を送信
                        tmux send-keys -t "$P_ID" "2" C-m
                    fi
                done
                echo "✅ ノードの再起動シグナルを送信しました！"
            else
                echo "⚠️ tmuxセッションが見つからないため再起動をスキップします。"
            fi
            
            # 再起動ループを防ぐためフラグをリセットしてループに戻る
            WAITING_FOR_RECONNECT=false
        fi
    fi

    sleep 2
done
