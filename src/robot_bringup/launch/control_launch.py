#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ─── package paths ──────────────────────────────────────────
    desc_share    = get_package_share_directory('robot_description')
    bringup_share = get_package_share_directory('robot_bringup')
    mux_share     = get_package_share_directory('twist_mux')
    slam_path = PathJoinSubstitution([
    bringup_share,
    'config',
    'mapper_params_online_sync.yaml'
    ])


    # ─── file paths ─────────────────────────────────────────────
    urdf_path        = PathJoinSubstitution([desc_share, 'urdf', 'robot.urdf.xacro'])
    cfg_path         = PathJoinSubstitution([bringup_share, 'config', 'robot_controllers.yaml'])
    rviz_config_path = PathJoinSubstitution([bringup_share, 'config', 'rvisconfig.rviz'])
    slam_config_path = PathJoinSubstitution([bringup_share, 'config', 'mapper_params_online_sync.yaml'])
    robot_desc       = Command(['xacro ', urdf_path])

    # ─── LiDAR launch (delayed 5 s so ros2_control comes up) ────
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    serial_baudrate   = LaunchConfiguration('serial_baudrate')
    frame_id          = LaunchConfiguration('frame_id')
    inverted          = LaunchConfiguration('inverted')
    angle_compensate  = LaunchConfiguration('angle_compensate')
    scan_mode         = LaunchConfiguration('scan_mode')

    lidar_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('sllidar_ros2'),
                        'launch', 'sllidar_a1_launch.py'
                    )
                ),
                launch_arguments={
                    'serial_port': lidar_serial_port,
                    'serial_baudrate': serial_baudrate,
                    'frame_id': frame_id,
                    'inverted': inverted,
                    'angle_compensate': angle_compensate,
                    'scan_mode': scan_mode,
                }.items()
            )
        ]
    )

    return LaunchDescription([
        # ─── Declare LiDAR args ─────────────────────────────────
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB_LIDAR'),
        DeclareLaunchArgument('serial_baudrate',   default_value='115200'),
        DeclareLaunchArgument('frame_id',          default_value='laser_frame'),
        DeclareLaunchArgument('inverted',          default_value='false'),
        DeclareLaunchArgument('angle_compensate',  default_value='true'),
        DeclareLaunchArgument('scan_mode',         default_value='Sensitivity'),

        lidar_launch,

        # ─── ros2_control ────────────────────────────────────────
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                cfg_path
            ],
        ),

        # ─── joint_state_broadcaster ─────────────────────────────
        Node(
            package='controller_manager',
            executable='spawner',
            name='joint_state_broadcaster_spawner',
            output='screen',
            arguments=['joint_state_broadcaster'],
            parameters=[cfg_path],
        ),

        # ─── robot_state_publisher ───────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),

        # ─── forward_command_controller ──────────────────────────
        Node(
            package='controller_manager',
            executable='spawner',
            name='forward_command_controller_spawner',
            output='screen',
            arguments=['forward_command_controller'],
            parameters=[cfg_path],
        ),

        # ─── SLAM-Toolbox in **sync** mode ───────────────────────
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config_path,
                {'use_sim_time': False},
            ],
            remappings=[('/scan', '/scan')],
        ),

        # delay SLAM start so odom->base_link is already published
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='sync_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[
                        slam_path,
                        {'publish_map_odom_transform': True},
                        {'transform_publish_period': 0.02},
                        {'scan_queue_size': 20},
                        {'transform_queue_size': 20},
                {       'transform_tolerance': 0.1},
                    ],
                    remappings=[('scan', '/scan')],
             )
        ]
        ),


        # ─── Lifecycle manager for SLAM ──────────────────────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['slam_toolbox'],
                'bond_timeout': 12.0,
            }]
        ),

        # ─── RViz ────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),

        # ─── teleop_twist_keyboard ───────────────────────────────
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            parameters=[{
                'scale_linear': 0.3,
                'scale_angular': 1.0,
                'stamped': True,
            }],
            remappings=[('/cmd_vel', '/cmd_vel_key')],
        ),

        # ─── joy_node ────────────────────────────────────────────
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }]
        ),

        # ─── teleop_twist_joy ────────────────────────────────────
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=[{'publish_stamped_twist': True}],
            remappings=[('/cmd_vel', '/cmd_vel_joy')],
        ),

        # ─── twist_mux ───────────────────────────────────────────
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[
                PathJoinSubstitution([mux_share, 'config', 'twist_mux_topics.yaml']),
                {
                    'cmd_vel_timeout':     0.5,
                    'cmd_vel_key_timeout': 0.5,
                    'cmd_vel_joy_timeout': 0.5,
                    'cmd_vel_mux_timeout': 0.5,
                },
            ],
            remappings=[('/cmd_vel_out', '/forward_command_controller/cmd_vel')],
        ),
    ])
