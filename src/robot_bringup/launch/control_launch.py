from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    desc_share    = get_package_share_directory('robot_description')
    bringup_share = get_package_share_directory('robot_bringup')

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

        # Load and configure ros2_control with controllers
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

        # Spawn joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            name='joint_state_broadcaster_spawner',
            output='screen',
        ),

        # Spawn forward_command_controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['forward_command_controller'],
            name='forward_command_controller_spawner',
            output='screen',
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
                'scale_angular': 1.0
            }],
            remappings=[
                ('/cmd_vel', '/forward_command_controller/cmd_vel')
            ]
        )
    ])
