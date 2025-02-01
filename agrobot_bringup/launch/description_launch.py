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

    base_footprint_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_static_tf',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '-0.075',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'
        ]
    )

    # Static tf of base_link -> odom
    odom_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_static_tf',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'odom', '--child-frame-id', 'base_link'
        ]
    )

    map_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_static_tf',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom'
        ]
    )

    laser_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_static_tf',
        output='screen',
        arguments=[
            '--x', '0.048', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'laser'
        ]
    )

    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_tf',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'imu_link'
        ]
    )

    ld = LaunchDescription()

    ld.add_action(odom_static_tf)
    ld.add_action(laser_static_tf)
    ld.add_action(imu_static_tf)
    ld.add_action(base_footprint_static_tf)
    ld.add_action(map_static_tf)

    return ld
