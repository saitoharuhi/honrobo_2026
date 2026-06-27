"""
shutdown_helper.py — 終了処理の確認ダイアログ

各ノードが KeyboardInterrupt (Ctrl+C) を受け取ったときに、
個別に終了するか、全ノードを一括で終了するかを選択できるようにします。
"""

import sys
import subprocess
import time


def ask_shutdown_action(node_name):
    """
    ユーザーに終了アクションを問い合わせる。
    戻り値:
        'y' : このノードのみ終了
        'a' : すべてのノードを終了
        'c' : キャンセル（再開）
    """
    # 画面クリアしてから選択肢を表示
    sys.stdout.write('\n\033[2J\033[H')
    sys.stdout.write("============================================\n")
    sys.stdout.write(f" ⚠️  [Ctrl+C] {node_name} が中断されました\n")
    sys.stdout.write("============================================\n")
    sys.stdout.write(" 以下の操作を選択してください:\n\n")
    sys.stdout.write("   [y] このプログラム（ノード）のみ終了する\n")
    sys.stdout.write("   [a] すべてのプログラム（全ノード）を終了する\n")
    sys.stdout.write("   [c] 中断をキャンセルして実行を継続する\n")
    sys.stdout.write("============================================\n")
    sys.stdout.flush()

    # 有効な入力があるまで繰り返す
    while True:
        try:
            choice = input("選択してください [y/a/c] (デフォルト: y): ").strip().lower()
            if not choice:
                return 'y'
            if choice in ('y', 'a', 'c'):
                return choice
            print("無効な入力です。y, a, c のいずれかを入力してください。")
        except (KeyboardInterrupt, EOFError):
            # プロンプト入力中にさらに Ctrl+C が押されたら、安全のためにこのノードのみ終了とする
            return 'y'


def trigger_stop_all():
    """すべてのノードを停止するスクリプトを実行"""
    print("\nすべてのプログラムを停止しています...")
    try:
        # stop_all.sh を実行
        # バックグラウンドで実行させ、自分自身が停止されるのを待つ
        subprocess.Popen(
            ['bash', '/home/haru/Documents/honrobo_2026/scripts/stop_all.sh'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        # 少し待ってから終了
        time.sleep(0.5)
    except Exception as e:
        print(f"一括停止スクリプトの実行に失敗しました: {e}")
