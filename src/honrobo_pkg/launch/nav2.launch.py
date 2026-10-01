import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
import serial.tools.list_ports

def find_lidar_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        # RPLIDAR (CP210x USB to UART Bridge) を検出
        if '10c4:ea60' in p.hwid.lower() or 'cp210' in p.description.lower():
            return p.device
    return '/dev/ttyUSB0'  # 見つからなかった場合のフォールバック

def generate_launch_description():
    pkg_share = get_package_share_directory('honrobo_pkg')

    # Launch Configurations
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_amcl = LaunchConfiguration('use_amcl')

    # Declare arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'map', 'map.yaml'),
        description='Full path to map yaml file to load'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_use_amcl_cmd = DeclareLaunchArgument(
        'use_amcl',
        default_value='true',
        description='Whether to enable AMCL for LiDAR self-position localization'
    )

    declare_use_lidar_cmd = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Whether to launch RPLIDAR S1 node'
    )

    # Lifecycle nodes to manage (without amcl)
    lifecycle_nodes_no_amcl = [
        'map_server',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator'
    ]

    # Lifecycle nodes to manage (with amcl)
    lifecycle_nodes_with_amcl = [
        'map_server',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
        'amcl'
    ]

    # Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml_file}
        ]
    )

    # AMCL Node (LiDAR 自己位置推定)
    amcl_node = Node(
        condition=IfCondition(use_amcl),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # Planner Server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # Controller Server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/nav_cmd')]
    )

    # Behavior Server
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # Lifecycle Manager (Without AMCL)
    lifecycle_manager_no_amcl_node = Node(
        condition=UnlessCondition(use_amcl),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_service',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': lifecycle_nodes_no_amcl}
        ]
    )

    # Lifecycle Manager (With AMCL)
    lifecycle_manager_with_amcl_node = Node(
        condition=IfCondition(use_amcl),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_service',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': lifecycle_nodes_with_amcl}
        ]
    )

    # Static Transform map -> odom (Only active when AMCL is OFF)
    static_tf_node = Node(
        condition=UnlessCondition(use_amcl),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        output='screen'
    )

    # RPLIDAR S1 Node
    use_lidar = LaunchConfiguration('use_lidar')
    rplidar_node = Node(
        condition=IfCondition(use_lidar),
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': find_lidar_port(),
            'serial_baudrate': 256000,      # S1のデフォルト
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen'
    )

    # Static Transform base_link -> laser (ロボット中心から前方へ0.475m, 高さ0.2m, 180度反転を適用)
    laser_tf_node = Node(
        condition=IfCondition(use_lidar),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['--x', '0.475', '--y', '0.0', '--z', '0.2',
                   '--yaw', '3.14159265', '--pitch', '0.0', '--roll', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        output='screen'
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_amcl_cmd)
    ld.add_action(declare_use_lidar_cmd)

    # Add Nodes
    ld.add_action(map_server_node)
    ld.add_action(amcl_node)
    ld.add_action(rplidar_node)
    ld.add_action(laser_tf_node)
    ld.add_action(planner_server_node)
    ld.add_action(controller_server_node)
    ld.add_action(behavior_server_node)
    ld.add_action(bt_navigator_node)
    ld.add_action(lifecycle_manager_no_amcl_node)
    ld.add_action(lifecycle_manager_with_amcl_node)
    ld.add_action(static_tf_node)
    ld.add_action(rviz_node)

    return ld
