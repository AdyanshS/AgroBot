import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('agrobot_vision'),
        'config',
        'agrobot_vision_params.yaml',
    )

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        output='screen',
        parameters=[{'video_device': '/dev/video2'}],
    )

    yolo_results_node = Node(
        package='agrobot_vision',
        executable='yolo_results.py',
        name='yolo_results',
        output='screen',
        parameters=[config],
    )

    ld = LaunchDescription()
    # Nodes
    ld.add_action(usb_cam_node)
    ld.add_action(yolo_results_node)

    return ld
