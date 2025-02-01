import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription


def generate_launch_description():

    lift_motor_control = Node(
        package='agrobot_hardware',
        executable='lift_motor_control.py',
        name='lift_motor_control',
        output='screen'
    )

    servo_control = Node(
        package='agrobot_hardware',
        executable='servo_control.py',
        name='servo_control',
        output='screen'
    )

    imu_driver_node = Node(
        package='agrobot_hardware',
        executable='imu_driver.py',
        name='bno080_driver',
        output='screen'
    )

    ld = LaunchDescription()

    # Nodes
    ld.add_action(lift_motor_control)
    ld.add_action(servo_control)
    ld.add_action(imu_driver_node)

    return ld
