from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='landmark_manager',
            executable='landmark_manager_node',
            name='landmark_manager'
        )
    ])