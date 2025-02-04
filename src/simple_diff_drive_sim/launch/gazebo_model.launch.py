#ROS2 and Gazebo Harmonic launch file for differential drive robot

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro


def generate_launch_description():
    
    robot_xacro_name='differential_drive_robot'   # needs to match robot name in Xacro file
    name_package = 'simple_diff_drive_sim' #name for package and paths
    model_file_relative_path='model/robot.xacro'   #relative path of robot.xacro
    world_file_relative_path='worlds/arena.world'

    path_model_file = os.path.join(get_package_share_directory(name_package), model_file_relative_path)   # absolute path model
    robot_description = xacro.process_file(path_model_file).toxml() # combines robot.xacro and robot.gazebo to xml to complete robot description
    path_world_file = os.path.join(get_package_share_directory(name_package), world_file_relative_path)  # absolute path to world  #uncomment for empty world

    # launch file from the gazebo_ros package itself
    gazebo_ros_package_launch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),
                                                                           'launch', 'gz_sim.launch.py'))
    
    #gazebo_launch=IncludeLaunchDescription(gazebo_ros_package_launch, launch_arguments={'gz_args': ['-r -v -v4', path_world_file], 'on_exit_shutdown': 'true'}.items())

    # uncomment this if for your using own world model
    gazebo_launch = IncludeLaunchDescription(
        gazebo_ros_package_launch,
        launch_arguments={
            'gz_args': f'-r -v4 {path_world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # spawn the model node in Gazebo
    spawn_model_node_gazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_xacro_name,
            '-topic', 'robot_description'
        ],
        output='screen',
    )

    # robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            #{'use_sim_time': LaunchConfiguration('use_sim_time')}
            {'use_sim_time': True}
        ]
    )

    # bridge gazebo and ros topics to enable control
    bridge_params = os.path.join(
        get_package_share_directory(name_package),
        'parameters',
        'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen'
    )

    arena_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_arena_odom',
        arguments=['0.6', '-0.6', '0', '0', '0', '0', 'arena', 'odom']
    )

    """ ground_truth = Node(
        package='simple_diff_drive_sim',
        executable='ground_truth_pose_publisher',
        output='screen',
    ) """

    launch_description_object = LaunchDescription() # create empty launch description
    launch_description_object.add_action(gazebo_launch)
    launch_description_object.add_action(spawn_model_node_gazebo)
    launch_description_object.add_action(robot_state_publisher_node)
    launch_description_object.add_action(start_gazebo_ros_bridge_cmd)
    launch_description_object.add_action(arena_tf)
    #launch_description_object.add_action(ground_truth)



    return launch_description_object