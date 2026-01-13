import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_robocon_gazebo = get_package_share_directory('robocon_gazebo')
    pkg_robocon_bringup = get_package_share_directory('robocon_bringup')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r ' + os.path.join(pkg_robocon_gazebo, 'worlds', 'robocon_2026.sdf')
        }.items(),
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={os.path.join(pkg_robocon_bringup, "config", "ros_gz_bridge.yaml")}'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
    ])
