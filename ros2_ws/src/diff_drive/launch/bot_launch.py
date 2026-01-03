from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'diff_drive',
            executable = 'velop',
            name = 'velocity_publisher'
        ),
        Node(
            package = 'diff_drive',
            executable = 'velos',
            name = 'velocity_subscriber'
        )
    ])