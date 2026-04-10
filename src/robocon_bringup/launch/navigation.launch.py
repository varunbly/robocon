import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    pkg_robocon_bringup = get_package_share_directory('robocon_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # 1. Start the simulation body (R1 and R2 skeleton)
    sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robocon_bringup, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Start the 3D EKFs (Locating robots in height and tilt)
    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robocon_bringup, 'launch', 'localization.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. Global Map Server (Shared by both robots)
    map_server = GroupAction([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_file}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': ['map_server']}]
        )
    ])

    # 4. R1 Navigation Stack
    r1_nav = GroupAction([
        PushRosNamespace('r1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'use_lifecycle_mgr': 'true',
                'autostart': 'true'
            }.items()
        )
    ])

    # 5. R2 Navigation Stack
    r2_nav = GroupAction([
        PushRosNamespace('r2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'use_lifecycle_mgr': 'true',
                'autostart': 'true'
            }.items()
        )
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_robocon_bringup, 'maps', 'map.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_robocon_bringup, 'config', 'nav2_params.yaml')),
        
        sim_cmd,
        localization_cmd,
        map_server,
        r1_nav,
        r2_nav
    ])
