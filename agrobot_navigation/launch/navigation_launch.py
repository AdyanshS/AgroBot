#!/usr/bin/python3

"""
A launch file that starts the Nav2 stack for robot navigation in a given map.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true',
        choices=['True', 'False']
    )

    # Get the share directory
    config_dir = os.path.join(
        get_package_share_directory('agrobot_navigation'), 'config')
    map_dir = os.path.join(
        get_package_share_directory('agrobot_navigation'), 'map')

    # Get the full path to the map file and params file
    map_file = os.path.join(map_dir, 'maze_map.yaml')
    param_file = os.path.join(config_dir, 'params.yaml')

    # Check if the files exist
    if not os.path.exists(map_file):
        raise FileNotFoundError(f"Map file not found: {map_file}")
    if not os.path.exists(param_file):
        raise FileNotFoundError(f"Params file not found: {param_file}")

    return LaunchDescription([
        use_sim_time_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'agrobot_navigation'), 'launch', 'nav2_bringup.launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'params_file': param_file,
                'use_sim_time': use_sim_time,
                'use_composition': 'True',
            }.items(),
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
