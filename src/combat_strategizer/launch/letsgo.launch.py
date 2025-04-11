import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    

    

    

    """
    # Include the slider_publisher launch file with the parameters
    slider_publisher = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slider_publisher'),
                'examples',
                'example.launch'
            )
        ),
        launch_arguments={'file': './config/Float32.yaml'}.items()
    )
    

    # Include the slider_publisher launch file with the parameters
    slider_publisher = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('utils'),
                'launch',
                'weaponspeed_slider.launch'
            )
        ),
        launch_arguments={'file': 'Float32.yaml'}.items()
    )
    """

    """
    # Add micro-ROS agent as a Node
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['udp4', '--port', '8888', '--dev', '	192.168.1.103']  #192.168.8.210
    )
    """

    map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    
    detection = Node(
        package='combat_strategizer',
        executable='main_lean',
        name='weapon_control',
    )
    

    combat_strategizer = Node(
        package='combat_strategizer',
        executable='simple_attack',
        name='simple_attack',
        parameters=[{'log_level': 'warn'}]  # Set the logging level to 'warn'
    )

    weapon_control = Node(
        package='combat_strategizer',
        executable='weapon_control',
        name='weapon_control',
    )

    simple_navigator = Node(
        package='robot_navigation',
        executable='simple_navigator',
        name='simple_navigator',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('robot_navigation'),
                'config',
                'params.yaml',
            ),
            {'log_level': 'warn'}]
    )

    # RViz2 launch node
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('robot_navigation'), 
            'config', 
            'rviz_config.rviz'  # Specify your RViz config file if you have one
        )]
    )


    # Build launch description
    ld = LaunchDescription()
    ld.add_action(map)
    ld.add_action(detection)
    ld.add_action(combat_strategizer)
    ld.add_action(weapon_control)
    ld.add_action(simple_navigator)
    ld.add_action(rviz2)

    return ld