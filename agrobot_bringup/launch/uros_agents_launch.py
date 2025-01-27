from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=[
                'serial',
                '--dev', '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_ac2e1ed5dc5aee118b5b6f0090fcc75d-if00-port0',
                '-b', '115200',
                '-v', '4'
            ]
        ),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent_udp',
            output='screen',
            arguments=[
                'udp4',
                '--port', '8888',
                '-v', '6'
            ]
        )
    ])
