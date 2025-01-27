import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription


def generate_launch_description():

    # Package Directories
    agrobot_vision_pkg_dir = get_package_share_directory('agrobot_vision')
    agrobot_bringup_pkg_dir = get_package_share_directory('agrobot_bringup')
    orbbec_camera_dir = get_package_share_directory('orbbec_camera')

    # Paths
    vision_params_file = os.path.join(
        agrobot_bringup_pkg_dir,
        'config',
        'object_tracking_params.yaml',
    )

    gemini_e_launch_file = os.path.join(
        orbbec_camera_dir, 'launch', 'gemini_e.launch.py'
    )

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        output='screen',
        parameters=[{'video_device': '/dev/video0'}],
    )

    yolo_results_node = Node(
        package='agrobot_vision',
        executable='yolo_results.py',
        name='yolo_results',
        output='screen',
        parameters=[vision_params_file],
    )

    object_tracking_node = Node(
        package='agrobot_vision',
        executable='object_tracking.py',
        name='object_tracking',
        output='screen',
        parameters=[vision_params_file],
    )

    orbec_gemini_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gemini_e_launch_file),
        launch_arguments={
            'camera_name': 'gemini_e',
            'depth_registration': 'false',
            'device_num': '1',
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
            'point_cloud_qos': 'default',
            'connection_delay': '100',
            'color_width': '640',
            'color_height': '360',
            'color_fps': '30',
            'color_format': 'MJPG',
            'enable_color': 'true',
            'flip_color': 'false',
            'color_qos': 'default',
            'color_camera_info_qos': 'default',
            'enable_color_auto_exposure': 'true',
            'cloud_frame_id': '',
            'color_exposure': '-1',
            'color_gain': '-1',
            'enable_color_auto_white_balance': 'true',
            'color_white_balance': '-1',
            'depth_width': '640',
            'depth_height': '360',
            'depth_fps': '30',
            'depth_format': 'Y11',
            'enable_depth': 'false',  # . true DEPTH CAM
            'flip_depth': 'false',
            'depth_qos': 'default',
            'depth_camera_info_qos': 'default',
            'ir_width': '640',
            'ir_height': '480',
            'ir_fps': '30',
            'ir_format': 'Y10',
            'enable_ir': 'false',  # .  true IR CAM
            'flip_ir': 'false',
            'ir_qos': 'default',
            'ir_camera_info_qos': 'default',
            'enable_ir_auto_exposure': 'true',
            'ir_exposure': '-1',
            'ir_gain': '-1',
            'publish_tf': 'true',
            'tf_publish_rate': '30.0',
            'ir_info_url': '',
            'color_info_url': '',
            'log_level': 'none',
            'enable_publish_extrinsic': 'false',
            'enable_d2c_viewer': 'false',
            'enable_ldp': 'true',
            'enable_soft_filter': 'true',
            'soft_filter_max_diff': '-1',
            'soft_filter_speckle_size': '-1',
            'ordered_pc': 'false',
            'use_hardware_time': 'false',
            'align_mode': 'HW',
            'laser_energy_level': '-1',
            'enable_heartbeat': 'false',
        }.items()
    )

    ld = LaunchDescription()
    # Nodes
    # ld.add_action(usb_cam_node)
    ld.add_action(yolo_results_node)
    ld.add_action(object_tracking_node)

    # Launches
    ld.add_action(orbec_gemini_launch)

    return ld
