from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    description_pkg_share = FindPackageShare('robocon_description')
    bringup_pkg_share = FindPackageShare('robocon_bringup')
    
    # Paths to files
    urdf_path = PathJoinSubstitution([description_pkg_share, 'models', 'rrr_robot', 'rrr_robot.urdf.xacro'])
    rviz_config_path = PathJoinSubstitution([bringup_pkg_share, 'config', 'rrr.rviz'])

    # Process URDF - "ParameterValue" is CRITICAL for Jazzy
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )
    
    # Bundle into a dictionary for easy reuse
    robot_params = {'robot_description': robot_description_content}

    return LaunchDescription([
        # 1. Publishes TF transforms (base_link -> link1 -> ...)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_params]
        ),
        # 2. GUI Slider - Needs robot_description to know joint limits
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[robot_params]
        ),
        # 3. RViz - Loads our saved config
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path]
        )
    ])
