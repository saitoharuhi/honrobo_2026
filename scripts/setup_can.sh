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

# カーネルモジュールのロード
echo ""
echo "[1/3] カーネルモジュールをロード中..."
modprobe slcan    2>/dev/null && echo "  ✅ slcan"    || echo "  ⚠️  slcan (ロード済みまたは不要)"
modprobe can      2>/dev/null && echo "  ✅ can"      || echo "  ⚠️  can (ロード済みまたは不要)"
modprobe can_raw  2>/dev/null && echo "  ✅ can_raw"  || echo "  ⚠️  can_raw (ロード済みまたは不要)"

# ── 1. 既存の SocketCAN デバイスチェック (candlelight / gs_usb / 既に起動中の場合) ──
echo ""
echo "[2/3] CANデバイスの検出・設定..."

if ip link show can0 &>/dev/null; then
    echo "  ℹ️  can0 インターフェースが存在します。"
    
    # 既に UP 状態か確認
    if ip link show can0 | grep -q "UP"; then
        ip link set can0 txqueuelen 1000 2>/dev/null || true
        echo "  ✅ can0 は既にアクティブ(UP)です。(txqueuelen 1000 適用済み)"
        for dev in /dev/ttyUSB* /dev/ttyACM*; do
            [ -e "$dev" ] && chmod 666 "$dev" 2>/dev/null || true
        done
        exit 0
    fi
    
    # DOWN 状態の場合、ネイティブ CAN (candlelight / gs_usb) として 1Mbps で UP を試みる
    if ip link set can0 type can bitrate 1000000 2>/dev/null; then
        ip link set can0 txqueuelen 1000 2>/dev/null || true
        ip link set can0 up 2>/dev/null || true
        if ip link show can0 | grep -q "UP"; then
            echo "  ✅ can0 を 1Mbps でアクティブ化しました (native SocketCAN)。"
            for dev in /dev/ttyUSB* /dev/ttyACM*; do
                [ -e "$dev" ] && chmod 666 "$dev" 2>/dev/null || true
            done
            exit 0
        fi
    fi
fi

# ── 2. SLCAN デバイスの自動検出 ──
echo "  CANable (SLCANシリアル) の自動検出中..."

CAN_PORT=$(python3 -c "
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
for p in ports:
    desc = p.description.lower()
    hwid = p.hwid.lower()
    if 'canable' in desc or '16d0:117e' in hwid:
        print(p.device)
        break
" 2>/dev/null || true)

if [ -z "$CAN_PORT" ]; then
    echo "  ⚠️  CANable名で未検出のため、/dev/ttyACM* から非STLinkデバイスを検索..."
    ACM_DEVICES=($(ls /dev/ttyACM* 2>/dev/null || true))
    for dev in "${ACM_DEVICES[@]}"; do
        DEV_NAME=$(basename "$dev")
        if [ -d "/sys/class/tty/$DEV_NAME/device" ]; then
            if ! grep -q -i "stlink" "/sys/class/tty/$DEV_NAME/device/interface" 2>/dev/null; then
                CAN_PORT="$dev"
                echo "  → 非STLinkデバイスをCANポートとして選択: $CAN_PORT"
                break
            fi
        fi
    done
fi

if [ -z "$CAN_PORT" ]; then
    # もし can0 が既にリンクとして存在していれば成功とする
    if ip link show can0 &>/dev/null; then
        ip link set can0 up 2>/dev/null || true
        if ip link show can0 | grep -q "UP"; then
            echo "  ✅ can0 が既に有効です。"
            exit 0
        fi
    fi
    echo "  ❌ CANableデバイスが見つかりません。"
    echo "     (CANアダプターが接続されているか確認してください)"
    exit 1
fi

echo "  ✅ CANableデバイスを検出しました: $CAN_PORT"

# 既存のcan0を停止し、slcand を再起動
ip link set can0 down 2>/dev/null || true
killall slcand 2>/dev/null || true
sleep 0.5

echo "$CAN_PORT" > /tmp/honrobo_can_port 2>/dev/null || true

slcand -o -c -s8 "$CAN_PORT" can0
ip link set can0 txqueuelen 1000
ip link set can0 up

# シリアルポートのパーミッション変更
echo ""
echo "[3/3] シリアルポートの書き込み権限を付与中..."
for dev in /dev/ttyUSB* /dev/ttyACM*; do
    if [ -e "$dev" ] && [ "$dev" != "$CAN_PORT" ]; then
        chmod 666 "$dev" 2>/dev/null || true
        echo "  ✅ $dev -> 権限 666 付与完了"
    fi
done

echo ""
echo "============================================"
echo " ✅ CAN0 セットアップ完了!"
echo "    デバイス: can0 (1Mbps, txqueuelen 1000)"
echo "    確認コマンド: candump can0"
echo "============================================"
