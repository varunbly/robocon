import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robocon_bringup = get_package_share_directory('robocon_bringup')

    # Parameters to be passed in from the caller (slam.launch or navigation.launch)
    use_sim_time = LaunchConfiguration('use_sim_time')
    configuration_basename = LaunchConfiguration('configuration_basename')

    # Core Cartographer SLAM node
    # Based on official backpack_3d pattern
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', os.path.join(pkg_robocon_bringup, 'config'),
            '-configuration_basename', configuration_basename
        ],
        remappings=[
            ('scan', 'scan'),
            ('odom', 'odom'),
            ('imu', 'imu'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Occupancy Grid node (Creates 2D map for Nav2)
    # Matches official pattern in backpack_3d.launch
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05'],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('configuration_basename', default_value='r1_lds_3d.lua'),
        
        cartographer_node,
        occupancy_grid_node
    ])
