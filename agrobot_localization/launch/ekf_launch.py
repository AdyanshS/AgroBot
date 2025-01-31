"""
This launch file initializes and launches the Extended Kalman Filter (EKF) node for robot localization.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf_config = PathJoinSubstitution([
        FindPackageShare("agrobot_localization"), "config", "ekf.yaml"
    ])

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        )
    ])
