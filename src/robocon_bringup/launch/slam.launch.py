import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    pkg_robocon_bringup = get_package_share_directory('robocon_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # 1. Start the simulation body (R1 and R2 skeleton)
    sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robocon_bringup, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Start Mapping (SLAM) for R1
    # We use the modular cartographer.launch and tell it to use R1's 3D logic
    r1_slam = GroupAction([
        PushRosNamespace('r1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_robocon_bringup, 'launch', 'cartographer.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'configuration_basename': 'r1_lds_3d.lua'
            }.items()
        )
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        sim_cmd,
        r1_slam
    ])
