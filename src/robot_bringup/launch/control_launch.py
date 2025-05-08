from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    desc_share    = get_package_share_directory('robot_description')
    bringup_share = get_package_share_directory('robot_bringup')

    urdf_path  = PathJoinSubstitution([desc_share,    'urdf',    'robot.urdf.xacro'])
    cfg_path   = PathJoinSubstitution([bringup_share, 'config',  'robot_controllers.yaml'])
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
        
        # Load and configure ros2_control w controllers
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',           # <-- changed here
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                cfg_path
            ],
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            name='joint_state_broadcaster_spawner',
            output='screen',
        ),

        
        # Launch forward command controller
        Node(
            package='controller_manager',
            executable='spawner',
            name='forward_command_controller',
            output='screen',
            arguments=['forward_command_controller'],
        ),

         # Launch teleop_twist_keyboard for keyboard control
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -e',  # Open in a separate terminal
            remappings=[
                ('/cmd_vel', '/forward_command_controller/cmd_vel')
            ]
        ),
     ])