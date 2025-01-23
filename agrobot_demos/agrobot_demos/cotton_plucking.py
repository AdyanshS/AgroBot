#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class CottonPlucking(Node):

    def __init__(self):
        super().__init__("cotton_plucking")
        example_param = self.declare_parameter("example_param", "default_value").value
        self.get_logger().info(f"Declared parameter 'example_param'. Value: {example_param}")
        self.get_logger().info("Hello world from the Python node cotton_plucking")


def main(args=None):
    rclpy.init(args=args)

    cotton_plucking = CottonPlucking()

    try:
        rclpy.spin(cotton_plucking)
    except KeyboardInterrupt:
        pass

    cotton_plucking.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
