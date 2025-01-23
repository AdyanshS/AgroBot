import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('agrobot_demos'),
        'config',
        'cotton_plucking_demo.yaml',
    )

    return LaunchDescription([
        Node(
            package='agrobot_demos',
            executable='cotton_plucking',
            name='cotton_plucking',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='agrobot_demos',
            executable='cotton_plucking',
            name='cotton_plucking',
            output='screen',
            parameters=[config],
        ),
    ])
