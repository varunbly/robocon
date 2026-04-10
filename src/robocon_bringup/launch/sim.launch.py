import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_robocon_gazebo = get_package_share_directory('robocon_gazebo')
    pkg_robocon_description = get_package_share_directory('robocon_description')
    pkg_robocon_bringup = get_package_share_directory('robocon_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # The directory containing the package (the 'share' directory)
    # This allows Gazebo to resolve package://robocon_description/...
    resource_path = os.path.join(pkg_robocon_description, 'models')

    # Append to the resource paths
    set_env_vars = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=resource_path, 
        separator=os.pathsep
    )
    set_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', 
        value=resource_path, 
        separator=os.pathsep
    )
    set_sdf_path = AppendEnvironmentVariable(
        name='SDF_PATH', 
        value=resource_path, 
        separator=os.pathsep
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r ' + os.path.join(pkg_robocon_gazebo, 'worlds', 'robocon_2026.sdf'),
            'on_exit_shutdown': 'True'
        }.items(),
    )

    # Pass environment variables explicitly to the nodes
    env_vars = os.environ.copy()
    def append_path(name, value):
        if name in env_vars and env_vars[name]:
            env_vars[name] = value + os.pathsep + env_vars[name]
        else:
            env_vars[name] = value

    append_path('GZ_SIM_RESOURCE_PATH', resource_path)
    append_path('IGN_GAZEBO_RESOURCE_PATH', resource_path)
    append_path('SDF_PATH', resource_path)

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={os.path.join(pkg_robocon_bringup, "config", "ros_gz_bridge.yaml")}'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        env=env_vars
    )

    # Robot State Publisher — using xacro
    xacro_path = os.path.join(pkg_robocon_description, 'models', 'R1', 'urdf', 'r1.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_desc}
        ],
        arguments=['--ros-args', '--log-level', 'info'],
        env=env_vars
    )

    # Joint State Publisher — publishes default (zero) states for the arm's
    # revolute/prismatic joints. Needed because the SDF model uses different
    # joint names than the URDF, so Gazebo's JointStatePublisher output
    # doesn't match the URDF joint names expected by robot_state_publisher.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        set_env_vars,
        set_ign_resource_path,
        set_sdf_path,
        gz_sim,
        bridge,
        robot_state_publisher,
        joint_state_publisher,
    ])