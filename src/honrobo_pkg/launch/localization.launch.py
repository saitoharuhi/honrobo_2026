import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('honrobo_pkg')
    map_yaml = LaunchConfiguration('map', default=os.path.join(pkg_dir, 'map', 'map_red.yaml'))

    return LaunchDescription([
        # ── TF: base_link → laser ──
        # LiDARはロボットの前方(X=0.475m)に180度反転(Yaw=3.14159)して取り付けられている
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_laser',
            arguments=['0.475', '0.0', '0.2', '3.14159265', '0.0', '0.0', 'base_link', 'laser'],
            output='screen'
        ),

        # ── 地図サーバー ──
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False}],
        ),

        # ── ライフサイクル管理 (map_server, amcl) ──
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),

        # ── LiDARスキャンフィルタ ──
        # ロボット後部の部品が写り込む部分を除去
        Node(
            package='honrobo_pkg',
            executable='scan_filter',
            name='scan_filter_node',
            output='screen',
            parameters=[{'front_angle_deg': 90.0}],
            remappings=[('/scan', '/scan')],
        ),

        # ── 標準自己位置推定 (nav2_amcl) ──
        # システムを簡略化・安定化させるため標準のAMCL（オムニ対応）を使用
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'robot_model_type': 'omnidirectional',
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'global_frame_id': 'map',
                'scan_topic': '/scan_filtered',
                'max_particles': 2000,
                'min_particles': 500,
                # ロボットの初期位置（デフォルト設定）
                'set_initial_pose': True,
                'initial_pose.x': 2.4,
                'initial_pose.y': -0.7,
                'initial_pose.yaw': 1.570796,
                'tf_broadcast': True,
            }],
        ),
    ])
