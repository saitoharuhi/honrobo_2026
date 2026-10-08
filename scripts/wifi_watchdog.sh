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
            echo "ROS 2 の stale 状態を解消するため、システムを自動再起動します..."
            
            # 3秒待つ（IPアドレス割り当てなどを確実にするため）
            sleep 3
            
            if tmux ls 2>/dev/null | grep -q "honrobo_operator"; then
                RESTART_SCRIPT="start_operator.sh"
            elif tmux ls 2>/dev/null | grep -q "honrobo_manual"; then
                RESTART_SCRIPT="start_manual.sh"
            else
                RESTART_SCRIPT="start_robot.sh"
            fi

            echo "🚀 $RESTART_SCRIPT を用いて再起動します..."
            
            # バックグラウンドでシステム再起動をトリガー
            nohup bash -c "bash $WORKSPACE_DIR/scripts/stop_all.sh && sleep 3 && bash $WORKSPACE_DIR/scripts/$RESTART_SCRIPT" >/dev/null 2>&1 &
            
            exit 0
        fi
    fi

    sleep 2
done
