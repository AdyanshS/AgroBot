import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription


def generate_launch_description():

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        choices=['True', 'False'],
        description='Use simulation (Gazebo) clock if true'
    )

    # Package Directories
    agrobot_bringup_pkg_dir = get_package_share_directory(
        'agrobot_bringup')

    # Launch file path to include
    actuator_launch_file = os.path.join(
        agrobot_bringup_pkg_dir, 'launch', 'actuator_control_launch.py')
    base_launch_file = os.path.join(
        agrobot_bringup_pkg_dir, 'launch', 'base_launch.py')
    vision_tracking_launch_file = os.path.join(
        agrobot_bringup_pkg_dir, 'launch', 'vision_tracking_launch.py')

    # Include Launch Description
    actuator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(actuator_launch_file))
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file))
    vision_tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vision_tracking_launch_file))

    ld = LaunchDescription()
    # Arguments
    ld.add_action(use_sim_time_arg)
    # Launch Files
    ld.add_action(actuator_launch)
    ld.add_action(base_launch)
    ld.add_action(vision_tracking_launch)

    return ld
