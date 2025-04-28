from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    desc_share    = get_package_share_directory('robot_description')
    bringup_share = get_package_share_directory('robot_bringup')

    urdf_path  = PathJoinSubstitution([desc_share,    'urdf',    'robot.urdf.xacro'])
    cfg_path   = PathJoinSubstitution([bringup_share, 'config', 'robot_controllers.yaml'])
    robot_desc = Command(['xacro ', urdf_path])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='ros2_control_node',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                cfg_path
            ],
        ),
    ])
