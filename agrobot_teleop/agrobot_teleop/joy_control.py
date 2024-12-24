#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class JoyControl(Node):

    def __init__(self):
        super().__init__("joy_control")
        example_param = self.declare_parameter("example_param", "default_value").value
        self.get_logger().info(f"Declared parameter 'example_param'. Value: {example_param}")
        self.get_logger().info("Hello world from the Python node joy_control")


def main(args=None):
    rclpy.init(args=args)

    joy_control = JoyControl()

    try:
        rclpy.spin(joy_control)
    except KeyboardInterrupt:
        pass

    joy_control.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
