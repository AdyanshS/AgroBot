#!/usr/bin/env python3

"""
   +-------+
   |       |
M1 \       / M4
    \     /
     \   /
      \ /
       +
      / \
     /   \
    /     \
M2 /       \ M3
   |       |
   +-------+
"""
import rclpy
import numpy as np
from math import pi, sin, cos, sqrt, tan
from rclpy.node import Node

from std_msgs.msg import String
from agrobot_interfaces.msg import MotorPWMs, EncoderPulses


class XDriveController(Node):

    def __init__(self):
        super().__init__("x_drive_controller_py")

        # Create Parameter
        self.declare_parameter("debug", False)
        self.debug = self.get_parameter("debug").value

        self.enc_sub_ = self.create_subscription(
            EncoderPulses,
            "encoder_pulses",
            self.encoder_pulses_callback,
            10
        )

        self.motor_pwm_pub_ = self.create_publisher(
            MotorPWMs,
            "motor_pwm",
            10
        )

        self.encoder_counts_: list[int] = [0, 0, 0, 0]

    def encoder_pulses_callback(self, msg: EncoderPulses):
        
        self.encoder_counts_[0] = msg.encoder_1_pulse
        self.encoder_counts_[1] = msg.encoder_2_pulse
        self.encoder_counts_[2] = msg.encoder_3_pulse
        self.encoder_counts_[3] = msg.encoder_4_pulse

        if self.debug:
            self.get_logger().info(
                f"Encoder Pulses: {self.encoder_counts_[0]}, {self.encoder_counts_[1]}, {self.encoder_counts_[2]}, {self.encoder_counts_[3]}"
            )

    def publish_motor_pwm(self, motor_pwms: list[int]):
        motor_pwm_msg = MotorPWMs()
        motor_pwm_msg.motor1pwm = motor_pwms[0]
        motor_pwm_msg.motor2pwm = motor_pwms[1]
        motor_pwm_msg.motor3pwm = motor_pwms[2]
        motor_pwm_msg.motor4pwm = motor_pwms[3]

        self.motor_pwm_pub_.publish(motor_pwm_msg)

        if self.debug:
            self.get_logger().info(
                f"Motor PWMs: {motor_pwms[0]}, {motor_pwms[1]}, {motor_pwms[2]}, {motor_pwms[3]}"
            )
        


def main(args=None):
    rclpy.init(args=args)

    x_drive_controller_py = XDriveController()

    try:
        rclpy.spin(x_drive_controller_py)
    except KeyboardInterrupt:
        pass

    x_drive_controller_py.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
