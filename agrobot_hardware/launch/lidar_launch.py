"""
This launch file is used to handle two LIDAR sensors and merging their scans.

Nodes:
- `rplidar_node_1`: A node from the `rplidar_ros` package responsible for
   handling the first RPLIDAR sensor.
- `rplidar_node_2`: A node from the `rplidar_ros` package responsible for
   handling the second RPLIDAR sensor.
- `scan_to_scan_filter_chain_1`: A node from the `laser_filters` package
  responsible for filtering the scan from the first LIDAR sensor.
- `scan_to_scan_filter_chain_2`: A node from the `laser_filters` package
   responsible for filtering the scan from the second LIDAR sensor.
- `merge_2_scan_launch.py`: An included launch file responsible for merging
   the filtered scans from both LIDAR sensors into a single scan topic.
"""


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    lidar_param_file = PathJoinSubstitution([
        get_package_share_directory("agrobot_hardware"),
        "config",
        "lidar.yaml"
    ])
    # Filter both lidar to a FOV of 270 degrees.
    laser_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain_1',
        parameters=[lidar_param_file],
        remappings=[
            ('/scan', '/scan/unfiltered'),
            ('/scan_filtered', '/scan')
        ]
    )

    # A2M12 lidar drivers
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node_1',
        parameters=[lidar_param_file],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(lidar_node)
    # ld.add_action(laser_filter)

    return ld
