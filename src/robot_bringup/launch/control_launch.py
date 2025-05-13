import os
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    desc_share    = get_package_share_directory('robot_description')
    bringup_share = get_package_share_directory('robot_bringup')
    teleop_share = get_package_share_directory('teleop_twist_joy')
    mux_share = get_package_share_directory('twist_mux')
    urdf_path  = PathJoinSubstitution([desc_share, 'urdf', 'robot.urdf.xacro'])
    cfg_path   = PathJoinSubstitution([bringup_share, 'config', 'robot_controllers.yaml'])
    robot_desc = Command(['xacro ', urdf_path])

    return LaunchDescription([

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
            parameters=[{
               # 'scale_linear': 0.3,
               # 'scale_angular': 1.0,
                'stamped': True,
                'config_filepath' : PathJoinSubstitution([teleop_share, 'config', 'ps4.config.yaml']),
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
