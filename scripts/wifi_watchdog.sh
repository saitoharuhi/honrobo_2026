#!/bin/bash
# Wi-Fi 自動復旧スクリプト (USB子機の抜き差し対応)

echo "=========================================="
echo " 📡 Wi-Fi 監視・自動復旧機能を起動しました"
echo " (5秒間隔で接続状態をチェックします)"
echo "=========================================="

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WAS_DISCONNECTED=false

while true; do
    WIFI_DEV=$(ip link | awk -F: '$0 !~ "lo|vir|eth|enp"{print $2;getline}' | grep -E 'wl|wlan' | tr -d ' ' | head -n 1)

    if [ -n "$WIFI_DEV" ]; then
        STATE=$(nmcli -t -f DEVICE,STATE dev | grep "^${WIFI_DEV}:" | cut -d':' -f2)

        if [ "$STATE" = "disconnected" ] || [ "$STATE" = "unavailable" ]; then
            WAS_DISCONNECTED=true
            echo "$(date '+%H:%M:%S') ⚠️ Wi-Fi ($WIFI_DEV) が切断されています。再接続を試みます..."
            nmcli device connect "$WIFI_DEV" > /dev/null 2>&1
            sleep 5
            
            NEW_STATE=$(nmcli -t -f DEVICE,STATE dev | grep "^${WIFI_DEV}:" | cut -d':' -f2)
            if [ "$NEW_STATE" = "disconnected" ]; then
                echo "$(date '+%H:%M:%S') 🔄 nmcli でネットワークをリセットします..."
                nmcli networking off
                sleep 2
                nmcli networking on
                sleep 10
            fi
        elif [ "$STATE" = "connected" ]; then
            if [ "$WAS_DISCONNECTED" = true ]; then
                echo "$(date '+%H:%M:%S') ✅ Wi-Fi ($WIFI_DEV) が復旧しました！"
                echo "ROS 2 の通信（Fast DDS）の stale 状態を解消するため、システムを自動再起動します..."
                
                # 動いているセッションを判定して再起動スクリプトを決定
                if tmux ls 2>/dev/null | grep -q "honrobo_operator"; then
                    RESTART_SCRIPT="start_operator.sh"
                elif tmux ls 2>/dev/null | grep -q "honrobo_manual"; then
                    RESTART_SCRIPT="start_manual.sh"
                else
                    RESTART_SCRIPT="start_robot.sh" # または start_all.sh
                    if [ -f "$WORKSPACE_DIR/scripts/start_robot.sh" ]; then
                        RESTART_SCRIPT="start_robot.sh"
                    else
                        RESTART_SCRIPT="start_all.sh"
                    fi
                fi

                echo "3秒後に $RESTART_SCRIPT を用いて再起動します..."
                sleep 3
                
                # 自身がtmux内で動いているため、バックグラウンドプロセスとして再起動を投げる
                nohup bash -c "bash $WORKSPACE_DIR/scripts/stop_all.sh && sleep 3 && bash $WORKSPACE_DIR/scripts/$RESTART_SCRIPT" >/dev/null 2>&1 &
                
                # このスクリプト自身は終了する（stop_allでいずれにせよkillされるため）
                exit 0
            fi
            WAS_DISCONNECTED=false
        fi
    else
        WAS_DISCONNECTED=true
        echo "$(date '+%H:%M:%S') 🔌 USB Wi-Fi子機が見つかりません。挿入されるのを待っています..."
    fi
    
    sleep 5
done
