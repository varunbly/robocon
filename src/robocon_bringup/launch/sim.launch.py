import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # -------------------------------------------------
    # Directories
    # -------------------------------------------------
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_robocon_description = get_package_share_directory('robocon_description')
    pkg_robocon_gazebo = get_package_share_directory('robocon_gazebo')

    # -------------------------------------------------
    # 1. Gazebo World
    # -------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r ' + os.path.join(pkg_robocon_gazebo, 'worlds', 'robocon_2026.sdf')
        }.items(),
    )

    # -------------------------------------------------
    # 2. Robot Description
    # -------------------------------------------------
    urdf_file = os.path.join(
        pkg_robocon_description, 'models', 'rrr_robot', 'rrr_robot.urdf.xacro'
    )
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # -------------------------------------------------
    # 3. State Publisher
    # -------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # -------------------------------------------------
    # 4. Spawn Robot
    # -------------------------------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'rrr_robot', '-z', '1.0'],
        output='screen'
    )

    # -------------------------------------------------
    # 5. Controller Spawners
    # -------------------------------------------------
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    # -------------------------------------------------
    # 6. Event Handlers
    # -------------------------------------------------
    start_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster],
        )
    )

    start_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller],
        )
    )

    # -------------------------------------------------
    # 7. Bridge (Clock Only)
    # -------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        robot_state_publisher,
        spawn_robot,
        start_jsb,
        start_arm_controller
    ])