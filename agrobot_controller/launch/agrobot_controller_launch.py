import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('agrobot_controller'),
        'config',
        'agrobot_controller_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='agrobot_controller',
            executable='x_drive_controller_cpp',
            name='x_drive_controller_cpp',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='agrobot_controller',
            executable='x_drive_controller_py',
            name='x_drive_controller_py',
            output='screen',
            parameters=[config],
        ),
    ])
