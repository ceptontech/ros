from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="cepton_publisher",
                executable="cepton_publisher_node",
                name="cepton_publisher_node",
            )
        ]
    )
