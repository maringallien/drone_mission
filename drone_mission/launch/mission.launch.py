from launch import LaunchDescription
from launch_ros.actions import Node


# Starts the mission_server node. Every parameter keeps its default from the node itself.
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_mission',
            executable='mission_server',
            name='mission_server',
            output='screen',
        ),
    ])
