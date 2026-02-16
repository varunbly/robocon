import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robocon_bringup = get_package_share_directory('robocon_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_robocon_description = get_package_share_directory('robocon_description')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # 1. Start the simulation (Gazebo, Bridge, Robot State Publisher)
    sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robocon_bringup, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Start Robot Localization (EKF)
    ekf_config_path = os.path.join(pkg_robocon_bringup, 'config', 'ekf.yaml')
    ekf_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 3. Start SLAM (mapping)
    slam_params_file = os.path.join(pkg_robocon_bringup, 'config', 'slam_params.yaml')
    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items()
    )

    # 4. Start RViz
    # resource_path is the parent of the package share directory
    resource_path = os.path.dirname(pkg_robocon_description)
    
    # Correct environment merging
    env_vars = os.environ.copy()
    def append_path(name, value):
        if name in env_vars and env_vars[name]:
            env_vars[name] = value + os.pathsep + env_vars[name]
        else:
            env_vars[name] = value

    append_path('GZ_SIM_RESOURCE_PATH', resource_path)
    append_path('IGN_GAZEBO_RESOURCE_PATH', resource_path)
    append_path('SDF_PATH', resource_path)
    append_path('GAZEBO_MODEL_PATH', resource_path)

    rviz_config_path = os.path.join(pkg_robocon_bringup, 'config', 'rrr.rviz')
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        env=env_vars
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(sim_cmd)
    ld.add_action(ekf_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(rviz_cmd)

    return ld
