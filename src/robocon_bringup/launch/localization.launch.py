import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    pkg_robocon_bringup = get_package_share_directory('robocon_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Path to individual EKF configs
    r1_ekf_config = os.path.join(pkg_robocon_bringup, 'config', 'r1_ekf.yaml')
    r2_ekf_config = os.path.join(pkg_robocon_bringup, 'config', 'r2_ekf.yaml')
    
    # R1 EKF (High precision 3D state)
    r1_ekf = GroupAction([
        PushRosNamespace('r1'),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[r1_ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        )
    ])

    # R2 EKF
    r2_ekf = GroupAction([
        PushRosNamespace('r2'),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[r2_ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        )
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        r1_ekf,
        r2_ekf
    ])
