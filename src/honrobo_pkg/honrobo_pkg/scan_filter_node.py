#!/usr/bin/env python3
"""
scan_filter_node.py — LiDARスキャンフィルタリングノード

ロボット後部（ロボット本体の部品が写り込む領域）の点群を除去し、
前方の点群のみをMCLに渡す。

レーザーフレームの角度関係:
  - LiDAR はロボット前方に逆向き取り付け → TF で yaw=π(180°) 補正済み
  - raw laserフレーム: 角度0° ≈ ロボット後方（部品あり）、±π ≈ ロボット前方
  - フィルタ: |angle| < front_angle_threshold の範囲を無効化（ロボット後方を除去）
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan


class ScanFilterNode(Node):
    def __init__(self):
        super().__init__('scan_filter_node')

        # フィルタ角度の設定
        # デフォルト: π/2(90°) → |angle| < 90° の範囲を無効化
        # → ロボット後方の90°±90°=180°分の点群を除去、前方180°のみ使用
        self.declare_parameter('front_angle_deg', 90.0)
        front_angle_deg = self.get_parameter('front_angle_deg').value
        self.threshold = math.radians(front_angle_deg)

        qos = qos_profile_sensor_data

        self.sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos)
        self.pub = self.create_publisher(
            LaserScan, '/scan_filtered', qos)

        self.get_logger().info(
            f'ScanFilterNode起動: 角度|θ| < {front_angle_deg}° の点を無効化 '
            f'(ロボット後方{front_angle_deg*2:.0f}°を除去)'
        )

    def _scan_cb(self, msg: LaserScan):
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max

        ranges = list(msg.ranges)
        intensities = list(msg.intensities) if msg.intensities else []

        for i, r in enumerate(ranges):
            angle = msg.angle_min + i * msg.angle_increment
            
            # 角度を -π ~ π に正規化
            norm_angle = math.atan2(math.sin(angle), math.cos(angle))

            # ロボット後方 (|norm_angle| < threshold) の点を無効化
            # (laserフレームの0度はロボットの真後ろを向いているため、0度付近を除去する)
            if abs(norm_angle) < self.threshold:
                ranges[i] = float('nan')
                if intensities:
                    intensities[i] = 0.0

        filtered.ranges = ranges
        if intensities:
            filtered.intensities = intensities

        self.pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
