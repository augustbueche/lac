from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='air_cleaning_pkg',
            executable='pm_sensor_publisher_node',
            output='screen'
        ),
#        Node(
#           package='hello_user1',
#            executable='subscriber',
#            output='screen'
#        )
    ])
