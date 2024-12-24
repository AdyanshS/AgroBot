from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory(
        'agrobot_teleop'), 'config', 'joystick_params.yaml')

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        # remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]
        remappings=[
                ('cmd_vel', 'cmd_vel/joy')]
        # ('cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]

    )

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
    )

    return LaunchDescription([
        joy_node,
        teleop_node
    ])
