#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from agrobot_interfaces.msg import ServoAngles
from std_msgs.msg import Float32
from numpy import clip


class ServoControl(Node):

    def __init__(self):
        super().__init__("servo_control")

        self.create_subscription(
            Float32, "servo_angle/claw", self.claw_callback, 10)
        self.create_subscription(
            Float32, "servo_angle/grip", self.grip_callback, 10)
        self.create_subscription(
            Float32, "servo_angle/arm", self.arm_callback, 10)

        self.servo_angles_pub = self.create_publisher(
            ServoAngles, "servo_angles", 10)

        self.servo_angles_timer = self.create_timer(
            0.1, self.servo_angles_timer_callback)

        self.servo_angles = ServoAngles()
        self.claw_angle: float = 0.0
        self.grip_angle: float = 0.0
        self.arm_angle: float = 0.0

        self.claw_servo_limits: tuple = (115, 260)
        self.grip_servo_limits: tuple = (150, 180)
        self.arm_servo_limits: tuple = (180, 300)

    def claw_callback(self, msg: Float32):
        """ Callback for claw angle subscriber """
        self.claw_angle = msg.data

    def grip_callback(self, msg: Float32):
        """ Callback for grip angle subscriber """
        self.grip_angle = msg.data

    def arm_callback(self, msg: Float32):
        """ Callback for arm angle subscriber """
        self.arm_angle = msg.data

    def servo_angles_timer_callback(self):
        """ Timer callback to publish servo angles """

        self.servo_angles.claw = clip(self.claw_angle, *self.claw_servo_limits)
        self.servo_angles.grip = clip(self.grip_angle, *self.grip_servo_limits)
        self.servo_angles.arm = clip(self.arm_angle, *self.arm_servo_limits)

        self.servo_angles_pub.publish(self.servo_angles)


def main(args=None):
    rclpy.init(args=args)

    servo_control = ServoControl()

    try:
        rclpy.spin(servo_control)
    except KeyboardInterrupt:
        pass

    servo_control.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
