import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():
  #package paths
  desc_share    = get_package_share_directory('robot_description')
  bringup_share = get_package_share_directory('robot_bringup')
  teleop_share = get_package_share_directory('teleop_twist_joy')
  mux_share = get_package_share_directory('twist_mux')

  #file paths
  urdf_path  = PathJoinSubstitution([desc_share, 'urdf', 'robot.urdf.xacro'])
  cfg_path   = PathJoinSubstitution([bringup_share, 'config', 'robot_controllers.yaml'])
  rviz_config_path = PathJoinSubstitution([bringup_share, 'config', 'rvisconfig.rviz'])
  slam_path = PathJoinSubstitution([bringup_share, 'config', 'mapper_params_online_async.yaml'])
  robot_desc = Command(['xacro ', urdf_path])
  
  # Launch configurations
  lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    #teensy_serial_port = LaunchConfiguration('teensy_serial_port')
  serial_baudrate = LaunchConfiguration('serial_baudrate')
  frame_id = LaunchConfiguration('frame_id')
  inverted = LaunchConfiguration('inverted')
  angle_compensate = LaunchConfiguration('angle_compensate')
  scan_mode = LaunchConfiguration('scan_mode')

    # LiDAR Launch
  lidar_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('sllidar_ros2'),
                        'launch',
                        'sllidar_a1_launch.py'
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
        # Declare launch arguments
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB_LIDAR'),
        #DeclareLaunchArgument('teensy_serial_port', default_value='/dev/ttyUSB_TEENSY'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('frame_id', default_value='laser_link'),
        DeclareLaunchArgument('inverted', default_value='false'),
        DeclareLaunchArgument('angle_compensate', default_value='true'),
        DeclareLaunchArgument('scan_mode', default_value='Sensitivity'),
        
        lidar_launch,



      # Launch robot state publisher
      Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          output='screen',
          parameters=[{'robot_description': robot_desc}],
      ),




      #Launch control node
      Node(
          package='controller_manager',
          executable='ros2_control_node',
          name='controller_manager',
          output='screen',
          parameters=[
              {'robot_description': robot_desc},
             PathJoinSubstitution([bringup_share, 'config', 'robot_controllers.yaml'])
          ],
       ),




      # Spawn joint_state_broadcaster
      Node(
          package='controller_manager',
          executable='spawner',
          arguments=['joint_state_broadcaster'],
          name='joint_state_broadcaster_spawner',
          output='screen',
           parameters=[
              {'robot_description': robot_desc},
              cfg_path
          ],
      ),




      # Spawn forward_command_controller
      Node(
          package='controller_manager',
          executable='spawner',
          arguments=['forward_command_controller'],
          name='forward_command_controller_spawner',
          output='screen',
          parameters=[
              {'robot_description': robot_desc},
              cfg_path
          ],
      ),
      
      # Launch SLAM Toolbox with predefined path
      Node(
          package='slam_toolbox',
          executable='async_slam_toolbox_node',  # or 'sync_slam_toolbox_node' if you're using synchronous mode
          name='slam_toolbox',
          output='screen',
          parameters=[slam_path],
          remappings=[
            
         ]
      ),

      Node(
          package='nav2_lifecycle_manager',
          executable='lifecycle_manager',
          name='lifecycle_manager_slam',
          output='screen',
          parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['slam_toolbox'],
          }]
    ),



      Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_laser_tf',
        arguments=[
        '--x', '0',
        '--y', '0',
        '--z', '0',
        '--roll', '0',
        '--pitch', '0',
        '--yaw', '0',
        '--frame-id', 'base_link',
        '--child-frame-id', 'laser_link'
        ],
        output='screen'
      ),


      # Launch RVIZ2
      Node(
          package='rviz2',
          executable='rviz2',
          name='rviz2',
          output='screen',
          arguments=['-d', rviz_config_path]
      ),


      # Launch teleop_twist_keyboard with remapping
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
          remappings=[
              ('/cmd_vel', '/cmd_vel_key')
          ]
      ),


      Node(
            package='ultrasonic_safety',
            executable='ultrasonic_safety_node',
            name='ultrasonic_safety_node',
            output='screen',
            parameters=[{
                #'scale_linear': 0.3,
                #'scale_angular': 1.0,
                'stamped': True,
            }],
            remappings=[
                ('/cmd_vel', '/cmd_vel_ultra')
            ]
      ),

      # Launch joy node
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




      # Launch teleop_twist_joy with remapping
      Node(
          package='teleop_twist_joy',
          executable='teleop_node',
          name='teleop_twist_joy',
          output='screen',
          parameters=[
              {'publish_stamped_twist': True,
        }],
          remappings=[
              ('/cmd_vel', '/cmd_vel_joy')
          ]
      ),




      # Launch twist_mux with remapping
      # This node will multiplex the cmd_vel topic from teleop_twist_keyboard and teleop_twist_joy
      # and publish to the /forward_command_controller/cmd_vel topic
      Node(
          package='twist_mux',
          executable='twist_mux',
          name='twist_mux',
          output='screen',
          parameters=[
              PathJoinSubstitution([mux_share, 'config', 'twist_mux_topics.yaml']),
            
              {
              'cmd_vel_timeout': 0.5,
              'cmd_vel_key_timeout': 0.5,
              'cmd_vel_joy_timeout': 0.5,
              'cmd_vel_mux_timeout': 0.5,
              #'use_stamped' : True,
              },
          ],
          remappings=[
              ('/cmd_vel_out', '/forward_command_controller/cmd_vel')
          ]
      )
  ])
