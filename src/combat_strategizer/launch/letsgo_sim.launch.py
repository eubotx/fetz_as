import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package share directories
    simple_diff_drive_sim_dir = get_package_share_directory('simple_diff_drive_sim')
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    
    # Static transform for map to odom
    static_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # Gazebo simulation
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simple_diff_drive_sim_dir,
                'launch',
                'gazebo_bot_with_enemy.launch.py'
            )
        )
    )

    # Combat strategizer node
    combat_strategizer = Node(
        package='combat_strategizer',
        executable='simple_attack',
        name='simple_attack',
        output='screen',
        parameters=[{'log_level': 'warn'}]
    )

    # Weapon control node
    weapon_control = Node(
        package='combat_strategizer',
        executable='weapon_control',
        name='weapon_control',
        output='screen'
    )

    # Navigation node
    simple_navigator = Node(
        package='robot_navigation',
        executable='simple_navigator',
        name='simple_navigator',
        output='screen',
        parameters=[
            os.path.join(
                robot_navigation_dir,
                'config',
                'params.yaml'
            ),
            {'log_level': 'warn'}
        ]
    )

    # RViz2 node
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            robot_navigation_dir, 
            'config', 
            'rviz_config.rviz'
        )]
    )

    # Include teleop launch file for joystick control
    teleop_enemy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('teleop_twist_joy'), 'launch', 'teleop-launch.py')
        ),
        launch_arguments={
            'joy_vel': 'enemy/cmd_vel',  # Remap velocity command topic
            'joy_config': 'xbox',   # Define joystick configuration
            #'joy_dev': '0',        # Joystick device (e.g., /dev/input/js0)
            #'publish_stamped_twist': 'false',  # Whether to publish a stamped twist
            #'config_filepath': 'config/bot135smallWheel_teleop.config.yaml'
        }.items()
    )

    return LaunchDescription([
        static_map_tf,
        gazebo_sim,
        combat_strategizer,
        weapon_control,
        simple_navigator,
        rviz2,
        teleop_enemy
    ])