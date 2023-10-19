from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_gui',
            executable='navigation_gui',
            name='navigation_gui'
        )
    ])