#!/bin/bash
# ============================================================
# USBシリアルポート（マイコン）の権限自動設定スクリプト
# ============================================================

echo "USBシリアルデバイスへのアクセス権限自動化用のudevルールを設定します..."
RULE_FILE="/etc/udev/rules.d/99-usb-serial.rules"

# ルールファイルの作成
echo 'KERNEL=="ttyACM*", MODE="0666"' | sudo tee "$RULE_FILE" > /dev/null
echo 'KERNEL=="ttyUSB*", MODE="0666"' | sudo tee -a "$RULE_FILE" > /dev/null

# udevルールの更新と適用
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "===================================================="
echo " ✅ udevルールの設定が完了しました！"
echo " 今後、マイコンを抜き差ししても権限エラーは発生しません。"
echo "===================================================="
