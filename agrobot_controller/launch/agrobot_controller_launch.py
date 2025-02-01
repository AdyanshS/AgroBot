import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    wheel_speed_control = Node(
        package='agrobot_controller',
        executable='wheel_speed_control.py',
        name='wheel_speed_control',
        output='screen',
    )

    x_drive_controller = Node(
        package='agrobot_controller',
        executable='x_drive_controller_py.py',
        name='x_drive_controller',
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(wheel_speed_control)
    ld.add_action(x_drive_controller)

    return ld
