from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="landmarks_file_name",
                default_value="",
                description="Name of the yaml file containing landmarks",
            ),
            Node(
                package="navigation_gui",
                executable="navigation_gui",
                name="navigation_gui",
                parameters=[
                    {
                        "landmarks_file_name": LaunchConfiguration(
                            "landmarks_file_name"
                        ),
                    }
                ],
            ),
        ]
    )
