import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_robocon_bringup = get_package_share_directory('robocon_bringup')

    # Path to the Nav2 parameters file
    nav2_params_file = os.path.join(pkg_robocon_bringup, 'config', 'nav2_params.yaml')
    
    # Path to the map file (Placeholder: You need to generate this map first!)
    map_file_path = os.path.join(pkg_robocon_bringup, 'maps', 'map.yaml')

    # Robot Localization (EKF)
    ekf_config_path = os.path.join(pkg_robocon_bringup, 'config', 'ekf.yaml')
    ekf_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path,
            {'use_sim_time': 'true'}
        ]
    )

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'map',
            default_value=map_file_path,
            description='Full path to map yaml file to load'
        ),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_params_file,
            description='Full path to the ROS2 parameters file to use for all launched nodes'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Start EKF
        ekf_cmd,

        # Include the standard Nav2 localization launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': 'true',  # Automatically transition nodes to Active
                'use_lifecycle_mgr': 'false' # localization_launch handles its own lifecycle manager
            }.items(),
        ),
    ])
