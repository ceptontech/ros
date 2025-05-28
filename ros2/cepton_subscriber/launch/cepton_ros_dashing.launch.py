from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="cepton_subscriber",
                executable="cepton_subscriber_node",
                name="cepton_subscriber_node",
            )
        ]
    )
