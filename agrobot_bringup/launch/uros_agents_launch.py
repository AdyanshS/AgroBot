from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    esp_s3_serial_id_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_ac2e1ed5dc5aee118b5b6f0090fcc75d-if00-port0'
    esp32_serial_id_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'

    esp_s3_serial_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='esps3_serial_agent',
        output='screen',
        arguments=[
            'serial',
            '--dev', esp_s3_serial_id_port,
            '-b', '115200',
            '-v', '4'
        ]
    )

    esp32_serial_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_serial',
        output='screen',
        arguments=[
            'serial',
            '--dev', esp32_serial_id_port,
            '-b', '115200',
            '-v', '4'
        ]
    )

    esp32_udp_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_udp',
        output='screen',
        arguments=[
                'udp4',
                '--port', '8888',
                '-v', '4'
        ]
    )

    return LaunchDescription([
        esp_s3_serial_agent,
        esp32_serial_agent
        # esp32_udp_agent
    ])
