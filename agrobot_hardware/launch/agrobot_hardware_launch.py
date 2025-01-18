import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('agrobot_hardware'),
        'config',
        'agrobot_hardware_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='agrobot_hardware',
            executable='servo_control',
            name='servo_control',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='agrobot_hardware',
            executable='servo_control',
            name='servo_control',
            output='screen',
            parameters=[config],
        ),
    ])
