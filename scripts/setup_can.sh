#!/bin/bash
# ============================================================
# CAN通信 初期設定スクリプト
# USB-CANアダプターのSocketCANインターフェースをセットアップします
# 
# 使い方: sudo bash scripts/setup_can.sh
# ============================================================

set -e

echo "============================================"
echo " CAN0 セットアップスクリプト"
echo "============================================"

# ACMデバイスの自動検出
echo ""
echo "[1/3] CANableデバイスを自動検出中..."

# Pythonを使ってCANableのシリアルポートを自動検出する
CAN_PORT=$(python3 -c "
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
for p in ports:
    desc = p.description.lower()
    hwid = p.hwid.lower()
    if 'canable' in desc or '16d0:117e' in hwid:
        print(p.device)
        break
" 2>/dev/null)

if [ -z "$CAN_PORT" ]; then
    echo "  ⚠️  CANableが自動検出されませんでした。/dev/ttyACM* からフォールバック検索します..."
    ACM_DEVICES=($(ls /dev/ttyACM* 2>/dev/null || true))
    if [ ${#ACM_DEVICES[@]} -eq 0 ]; then
        echo "  ❌ /dev/ttyACM* が見つかりません。"
        echo "     USB-CANアダプターが接続されているか確認してください。"
        exit 1
    elif [ ${#ACM_DEVICES[@]} -eq 1 ]; then
        CAN_PORT="${ACM_DEVICES[0]}"
        echo "  → 唯一のACMデバイスをCANポートとして選択: $CAN_PORT"
    else
        # 複数ある場合はSTLinkではない方を優先して探す
        for dev in "${ACM_DEVICES[@]}"; do
            DEV_NAME=$(basename "$dev")
            if [ -d "/sys/class/tty/$DEV_NAME/device" ]; then
                # STLinkなどのシリアルポートであるか簡易チェック
                if ! grep -q -i "stlink" "/sys/class/tty/$DEV_NAME/device/interface" 2>/dev/null; then
                    CAN_PORT="$dev"
                    echo "  → 非STLinkデバイスをCANポートとして自動選択: $CAN_PORT"
                    break
                fi
            fi
        done
        if [ -z "$CAN_PORT" ]; then
            CAN_PORT="${ACM_DEVICES[0]}"
            echo "  → フォールバックとして最初のデバイスを選択: $CAN_PORT"
        fi
    fi
else
    echo "  ✅ CANableデバイスを自動検出しました: $CAN_PORT"
fi

echo ""
echo "[2/3] カーネルモジュールをロード中..."
modprobe slcan  2>/dev/null && echo "  ✅ slcan"  || echo "  ⚠️  slcan (既にロード済み)"
modprobe can    2>/dev/null && echo "  ✅ can"    || echo "  ⚠️  can (既にロード済み)"
modprobe can_raw 2>/dev/null && echo "  ✅ can_raw" || echo "  ⚠️  can_raw (既にロード済み)"

echo ""
echo "[3/3] SocketCANインターフェースを起動中..."

# 既存のcan0を一旦停止（エラーは無視）
ip link set can0 down 2>/dev/null || true
# 既存のslcandプロセスを停止
killall slcand 2>/dev/null || true
sleep 0.5

# 使用中のCANポートを記録（zikoiti_nodeでの誤認防止用）
echo "$CAN_PORT" > /tmp/honrobo_can_port || true

slcand -o -c -s8 "$CAN_PORT" can0
ip link set can0 up

# シリアルポートのパーミッション変更
echo ""
echo "[4/4] シリアルポートの書き込み権限を付与中..."
for dev in /dev/ttyUSB* /dev/ttyACM*; do
    if [ -e "$dev" ] && [ "$dev" != "$CAN_PORT" ]; then
        chmod 666 "$dev"
        echo "  ✅ $dev -> 権限 666 付与完了"
    fi
done

echo ""
echo "============================================"
echo " ✅ CAN0 セットアップ完了!"
echo "    デバイス: $CAN_PORT → can0"
echo "    ボーレート: 1Mbps (s8)"
echo ""
echo "    確認コマンド: candump can0"
echo "============================================"
