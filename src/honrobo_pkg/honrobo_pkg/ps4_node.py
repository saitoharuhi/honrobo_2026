"""
ps4_node.py — PS4コントローラー入力ノード

配信トピック: /ps4_joy (sensor_msgs/Joy)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame
import sys

BUTTON_MAP = {
    0: 'Square', 1: 'Cross', 2: 'Circle', 3: 'Triangle',
    4: 'L1', 5: 'R1', 6: 'L2(Btn)', 7: 'R2(Btn)',
    8: 'SHARE', 9: 'OPTIONS', 10: 'PS',
    11: 'L3', 12: 'R3', 13: 'UP', 14: 'DOWN', 15: 'LEFT', 16: 'RIGHT',
}

DEADZONE = 0.08


class Ps4Node(Node):
    def __init__(self):
        super().__init__('ps4_node')
        self.pub = self.create_publisher(Joy, 'ps4_joy', 10)
        # ヘッドレス環境(SSH等)でもイベントループが動くようにダミードライバを設定
        import os
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        self._connect()
        
        # L2 / R2 初期値補正用フラグ
        self.l2_initialized = False
        self.r2_initialized = False

        self.create_timer(0.02, self._read_input)
        self.create_timer(0.1, self._print_display)
        self.axes_disp = [0.0] * 6
        self.btns_disp = []

    def _connect(self):
        pygame.joystick.quit()
        pygame.joystick.init()
        count = pygame.joystick.get_count()
        self.joystick = None

        for i in range(count):
            try:
                joy = pygame.joystick.Joystick(i)
                joy.init()
                name = joy.get_name().lower()
                # PS4コントローラーを表す名称にマッチ
                if "wireless controller" in name or "sony" in name or "playstation" in name:
                    self.joystick = joy
                    self.get_logger().info(f"Connected to PS4 Controller: {joy.get_name()} (Device {i})")
                    break
                else:
                    joy.quit()
            except Exception:
                pass

        if not self.joystick and count > 0:
            try:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.get_logger().warn(f"PS4 Controller name not matched. Fallback to device 0: {self.joystick.get_name()}")
            except Exception as e:
                self.joystick = None

    def _read_input(self):
        pygame.event.pump()
        if not self.joystick:
            self._connect()
            if not self.joystick:
                return
        try:
            num_axes = min(self.joystick.get_numaxes(), 6)
            raw_axes = []
            for i in range(num_axes):
                raw_axes.append(self.joystick.get_axis(i))
            while len(raw_axes) < 6:
                raw_axes.append(0.0)

            # 軸の並び替え (ユーザー環境のキーマップに対応)
            # raw_axes[2] (物理RY), raw_axes[3] (物理L2), raw_axes[4] (物理RX)
            axes = [0.0] * 6
            axes[0] = raw_axes[0]  # LX
            axes[1] = raw_axes[1]  # LY
            axes[2] = raw_axes[3]  # RX
            axes[3] = raw_axes[4]  # RY
            axes[4] = raw_axes[2]  # L2
            axes[5] = raw_axes[5]  # R2

            # LY と RY の符号反転 (上が正になるように)
            axes[1] = -axes[1]
            axes[3] = -axes[3]

            # スティック(LX, LY, RX, RY)のデッドゾーン適用
            for i in range(4):
                if abs(axes[i]) < DEADZONE:
                    axes[i] = 0.0

            # L2, R2 のアナログ出力を 0.0 ~ 1.0 にスケーリング
            # 初期値が 0.0 の場合、トリガーが触られていない状態(-1.0)とみなす
            l2_raw = axes[4]
            if not self.l2_initialized:
                if l2_raw != 0.0:
                    self.l2_initialized = True
                else:
                    l2_raw = -1.0

            r2_raw = axes[5]
            if not self.r2_initialized:
                if r2_raw != 0.0:
                    self.r2_initialized = True
                else:
                    r2_raw = -1.0

            axes[4] = (l2_raw + 1.0) / 2.0
            axes[5] = (r2_raw + 1.0) / 2.0

            buttons = [self.joystick.get_button(i)
                       for i in range(self.joystick.get_numbuttons())]
            if self.joystick.get_numhats() > 0:
                hat = self.joystick.get_hat(0)
                buttons += [
                    1 if hat[1] > 0 else 0,
                    1 if hat[1] < 0 else 0,
                    1 if hat[0] < 0 else 0,
                    1 if hat[0] > 0 else 0,
                ]

            self.axes_disp = axes[:]
            self.btns_disp = [BUTTON_MAP.get(i, f'B{i}')
                              for i, v in enumerate(buttons) if v == 1]

            msg = Joy()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.axes = [float(a) for a in axes]
            msg.buttons = buttons
            self.pub.publish(msg)
        except pygame.error as e:
            self.get_logger().warn(f"Controller error: {e}")
            self.joystick = None

    def _print_display(self):
        sys.stdout.write('\033[2J\033[H')
        conn = "Connected" if self.joystick else "Disconnected"
        sys.stdout.write(f"==== PS4 NODE | {conn} ====\n")
        for i, lbl in enumerate(['LX', 'LY', 'RX', 'RY', 'L2', 'R2']):
            v = self.axes_disp[i] if i < len(self.axes_disp) else 0.0
            sys.stdout.write(f"  {lbl}: {v:>6.2f}\n")
        btns = ", ".join(self.btns_disp) if self.btns_disp else "None"
        sys.stdout.write(f"[BUTTONS] {btns}\n")
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = Ps4Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

