#!/bin/bash
# Wi-Fi 自動復旧スクリプト (USB子機の抜き差し対応)

echo "=========================================="
echo " 📡 Wi-Fi 監視・自動復旧機能を起動しました"
echo " (5秒間隔で接続状態をチェックします)"
echo "=========================================="

while true; do
    # 'wl' または 'wlan' から始まるWi-Fiデバイス名を取得
    WIFI_DEV=$(ip link | awk -F: '$0 !~ "lo|vir|eth|enp"{print $2;getline}' | grep -E 'wl|wlan' | tr -d ' ' | head -n 1)

    if [ -n "$WIFI_DEV" ]; then
        # Wi-Fiデバイスが存在する場合（子機が刺さっている）
        
        # 接続状態を確認
        STATE=$(nmcli -t -f DEVICE,STATE dev | grep "^${WIFI_DEV}:" | cut -d':' -f2)

        if [ "$STATE" = "disconnected" ] || [ "$STATE" = "unavailable" ]; then
            echo "$(date '+%H:%M:%S') ⚠️ Wi-Fi ($WIFI_DEV) が切断されています。再接続を試みます..."
            
            # 再接続コマンド (sudo不要なnmcliを使用)
            nmcli device connect "$WIFI_DEV" > /dev/null 2>&1
            
            sleep 5 # 接続試行のため待機
            
            # それでもダメならネットワーク全体の再起動を試みる (パスワード不要な場合)
            NEW_STATE=$(nmcli -t -f DEVICE,STATE dev | grep "^${WIFI_DEV}:" | cut -d':' -f2)
            if [ "$NEW_STATE" = "disconnected" ]; then
                echo "$(date '+%H:%M:%S') 🔄 nmcli でネットワークをリセットします..."
                nmcli networking off
                sleep 2
                nmcli networking on
                sleep 10
            fi
        fi
    else
        # デバイスが存在しない（子機が抜かれている）
        echo "$(date '+%H:%M:%S') 🔌 USB Wi-Fi子機が見つかりません。挿入されるのを待っています..."
    fi
    
    sleep 5
done
