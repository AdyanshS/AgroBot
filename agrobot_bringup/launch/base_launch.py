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
    agrobot_controller_pkg_dir = get_package_share_directory(
        'agrobot_controller')
    agrobot_teleop_pkg_dir = get_package_share_directory('agrobot_teleop')

    twist_mux_params = os.path.join(get_package_share_directory(
        'agrobot_teleop'), 'config', 'twist_mux.yaml')

    # Launch file to include
    agrobot_controller_path = os.path.join(
        agrobot_controller_pkg_dir, 'launch', 'agrobot_controller_launch.py')

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[
            twist_mux_params],
        remappings=[
            ('cmd_vel_out', 'cmd_vel/drive')
        ]
    )

    agrobot_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(agrobot_controller_path),
    )

    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            agrobot_teleop_pkg_dir, 'launch', 'joystick.launch.py')),
    )

    # Static tf of base_link to odom
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'odom'],
        output='screen'
    )

    ld = LaunchDescription()
    # Arguments
    ld.add_action(use_sim_time_arg)
    # Nodes
    ld.add_action(twist_mux_node)
    # ld.add_action(static_tf)
    # Launch Files
    ld.add_action(agrobot_controller_launch)
    ld.add_action(joystick_launch)

    return ld
