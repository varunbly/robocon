from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_robocon_description = get_package_share_directory('robocon_description')
    
    # Spawn assembly robot in Gazebo
    spawn_assembly_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'assembly_robot',
            '-topic', 'robot_description',
            '-x', '2.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    return LaunchDescription([
        spawn_assembly_robot
    ])
