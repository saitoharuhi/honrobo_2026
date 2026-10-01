"""
localization.launch.py
holo_mcl (natto_library) を使用する自己位置推定ランチファイル。

パイプライン:
  /scan (LaserScan)
    → scan_filter_node (/scan_filtered: ロボット後方を除去)
    → laserscan_to_pointcloud2 (/pointcloud2: base_linkフレームに変換)
    → holo_mcl (MCL自己位置推定 → map→base_link TF)
"""
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('honrobo_pkg')
    map_yaml   = LaunchConfiguration('map',        default=os.path.join(pkg_dir, 'map', 'map_red.yaml'))
    mcl_params = LaunchConfiguration('mcl_params', default=os.path.join(pkg_dir, 'config', 'mcl_params.yaml'))

    return LaunchDescription([

        # ── 地図サーバー ──
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False}],
        ),

        # ── ライフサイクル管理 (map_server のみ) ──
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),

        # ── ① LiDARスキャンフィルタ ──
        # ロボット後部（robot body が写り込む領域）の点群を除去する
        # front_angle_deg: この角度未満の範囲（laser frameで0°=robot後方）を無効化
        # 90° → ロボット後方180°を除去、前方180°のみ使用
        Node(
            package='honrobo_pkg',
            executable='scan_filter',
            name='scan_filter_node',
            output='screen',
            parameters=[{'front_angle_deg': 90.0}],
            remappings=[('/scan', '/scan')],
        ),

        # ── ② LaserScan → PointCloud2 変換 ──
        # frame_id='base_link' にすることで、LiDARの取り付けオフセット(x=0.475m, yaw=π)
        # を TF 経由で自動補正した座標系でMCLに渡す
        Node(
            package='holo_lidar_converter',
            executable='laserscan_to_pointcloud2',
            name='laserscan_to_pointcloud2',
            output='screen',
            parameters=[{'frame_id': 'base_link'}],
            remappings=[
                ('laserscan',   '/scan_filtered'),
                ('pointcloud2', '/pointcloud2'),
            ],
        ),

        # ── ③ holo_mcl 自己位置推定 ──
        Node(
            package='holo_mcl',
            executable='mcl',
            name='mcl',
            output='screen',
            parameters=[mcl_params],
            remappings=[
                ('occupancy_grid',    '/map'),
                ('pointcloud2',       '/pointcloud2'),
                ('odometry',          '/odom'),
                ('pose',              '/localization/pose'),
                ('particles',         '/localization/particles'),
                ('pose_with_covariance', '/localization/pose_with_covariance'),
                ('initial_pose',      '/initialpose'),
            ],
        ),
    ])
