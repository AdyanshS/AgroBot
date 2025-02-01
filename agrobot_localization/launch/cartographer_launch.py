"""
This launch file initializes and launches Cartographer ROS nodes for SLAM.

Nodes:
- `cartographer_node`: A node from the `cartographer_ros`
   package responsible for SLAM.
- `cartographer_occupancy_grid_node`: A node from the `cartographer_ros`
   package responsible for generating an occupancy grid map.

Arguments:
- `use_sim_time`: Boolean parameter to use simulation (Gazebo) clock if true.
- `configuration_directory`: Directory containing the Cartographer
   configuration files.
- `configuration_basename`: Basename of the Cartographer configuration file.
- `resolution`: Resolution of the occupancy grid map.
- `publish_period_sec`: Period in seconds to publish the occupancy grid map.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    # Launch Configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        choices='[True, False]',
        description='Use simulation (Gazebo) clock if true')

    cartographer_config_dir = os.path.join(
        get_package_share_directory('agrobot_localization'), 'config')
    configuration_basename = 'cartographer.lua'  # configuration file

    return LaunchDescription([
        # Launch Arguments
        use_sim_time_arg,

        # Launch nodes
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
            # remappings=[('scan', '/scan_fullframe')],
            remappings=[('/odom', '/odometry/filtered')],
            # remappings=[('/odom', '/diffdrive/odom')]

        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
    ])
