import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Get paths (good for portability of code)
    pkg_path = get_package_share_directory('diff_drive')
    world_path = os.path.join(pkg_path, 'worlds/test_world.sdf')
    bot_model_path = os.path.join(pkg_path, 'models/diff_drive_bot/diff_drive.sdf')

    return LaunchDescription([

        # Launch gazebo with the custom world    
        ExecuteProcess(
            cmd = ['gz', 'sim', world_path], # cmd = [executable, arg1, arg2, ...]
            output = 'screen'
        ),

        # Launch the differential drive bot in the custom world
        Node(
            package = 'ros_gz_sim',
            executable = 'create', # Used to place the model into the world
            arguments = ['-file', bot_model_path, '-name', 'diff_drive_bot'],
                # Valued arguments (ex: -name 'my_node'):
                # arguments = [flag1, value1, flag2, value2]
                # Bool arguments (ex: -allow_renaming):
                # arguments = [flag]
            output = 'screen'
        ),

        # Publisher node
        Node(
            package = 'diff_drive',
            executable = 'velop',
            name = 'velocity_publisher'
        ),

        # Subscriber node
        Node(
            package = 'diff_drive',
            executable = 'velos',
            name = 'velocity_subscriber'
        )
    ])