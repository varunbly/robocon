import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer

def generate_launch_description():
    pkg_robocon_bringup = get_package_share_directory('robocon_bringup')
    pkg_robocon_description = get_package_share_directory('robocon_description')
    
    # use_sim_time = LaunchConfigAsBool('use_sim_time')
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

    # 3. Start Cartographer SLAM [NEW]
    # Configuration directory should be the config folder in your bringup package
    cartographer_config_dir = os.path.join(pkg_robocon_bringup, 'config')
    cartographer_config_basename = 'r1_lds_2d.lua'

    cartographer_cmd = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_basename
        ],
        remappings=[('/imu', '/imu')]
    )

    # 3b. Start Occupancy Grid Node (Converts Cartographer submaps to a /map topic)
    occupancy_grid_cmd = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    # 4. Start Kinematics Node (The Glue Code) [NEW ADDITION]
    # This runs the script you added to robocon_application/setup.py
    kinematics_cmd = Node(
        package='robocon_application',   # Must match package name in package.xml
        executable='simple_kinematics',  # Must match entry point name in setup.py
        name='kinematics_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 5. Start RViz
    # resource_path is the parent of the package share directory
    resource_path = os.path.join(pkg_robocon_description, 'models')
    
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

    rviz_config_path = os.path.join(pkg_robocon_bringup, 'config', 'r1_nav.rviz')
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
    ld.add_action(cartographer_cmd)
    ld.add_action(occupancy_grid_cmd)
    ld.add_action(kinematics_cmd) 
    ld.add_action(rviz_cmd)

    return ld