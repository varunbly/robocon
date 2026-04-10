import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_robocon_description = get_package_share_directory('robocon_description')
    pkg_robocon_bringup = get_package_share_directory('robocon_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Environment variables for Gazebo
    resource_path = os.path.join(pkg_robocon_description, 'models')
    
    # 1. Gazebo Server & World
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r ' + os.path.join(get_package_share_directory('robocon_gazebo'), 'worlds', 'robocon_2026.sdf'),
        }.items(),
    )

    # 2. Shared ROS-GZ Bridge (Handles all namespaced topics)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={os.path.join(pkg_robocon_bringup, "config", "ros_gz_bridge.yaml")}'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 3. R1 "Body" Setup (Namespace & State Publisher)
    r1_xacro_path = os.path.join(pkg_robocon_description, 'models', 'R1', 'urdf', 'r1.urdf.xacro')
    r1_robot_desc = ParameterValue(Command(['xacro ', r1_xacro_path]), value_type=str)
    
    r1_body = GroupAction([
        PushRosNamespace('r1'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{
                'use_sim_time': use_sim_time, 
                'robot_description': r1_robot_desc,
                'frame_prefix': 'r1/'  # Ensures TF frames are r1/base_link
            }]
        )
    ])

    # 4. R2 "Body" Setup (Namespace & State Publisher)
    # Using the same model but independent publisher/namespace
    r2_xacro_path = os.path.join(pkg_robocon_description, 'models', 'R1', 'urdf', 'r1.urdf.xacro') 
    r2_robot_desc = ParameterValue(Command(['xacro ', r2_xacro_path]), value_type=str)
    
    r2_body = GroupAction([
        PushRosNamespace('r2'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[{
                'use_sim_time': use_sim_time, 
                'robot_description': r2_robot_desc,
                'frame_prefix': 'r2/' # Ensures TF frames are r2/base_link
            }]
        )
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gz_sim,
        bridge,
        r1_body,
        r2_body
    ])