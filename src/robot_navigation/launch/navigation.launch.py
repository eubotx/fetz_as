import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_navigation'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='robot_navigation',
            executable='simple_navigator',
            name='simple_navigator',
            output='screen',
            parameters=[config]
        )
    ])