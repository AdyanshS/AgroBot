#!/usr/bin/env python3
from math import e
import rclpy
from rclpy.node import Node

from agrobot_interfaces.msg import LimitSwitchStates
from std_msgs.msg import Int32


class LiftMotorControlNode(Node):

    def __init__(self):
        super().__init__('lift_motor_control_node')

        self.create_subscription(
            LimitSwitchStates, 'limit_switch_states', self.limit_switch_states_callback, 10)
        self.create_subscription(
            Int32, 'lift_direction', self.lift_direction_callback, 10)

        self.lift_motor_pwm_publisher = self.create_publisher(
            Int32, 'lift_motor_pwm', 10)

        self.limit_switch_1 = False
        self.limit_switch_2 = False
        self.lift_direction = 0

        self.lift_motor_pwm = 150  # PWM value for lift motor

        self.lift_motor_timer = self.create_timer(
            0.3, self.lift_motor_timer_callback)

    def limit_switch_states_callback(self, msg: LimitSwitchStates):
        """Callback for limit switch states subscriber"""
        self.limit_switch_1 = msg.limit_switch_1
        self.limit_switch_2 = msg.limit_switch_2

    def lift_direction_callback(self, msg: Int32):
        """Callback for lift direction subscriber"""
        self.lift_direction = msg.data

    def lift_motor_timer_callback(self):
        if self.limit_switch_1 or self.limit_switch_2:
            # Stop lift motor if limit switch is pressed
            self.publish_lift_motor_pwm(0)
        elif self.lift_direction == 1:
            # Go up
            self.publish_lift_motor_pwm(200)
            self.lift_direction = 0  # Reset direction after processing
        elif self.lift_direction == -1:
            # Go down
            self.publish_lift_motor_pwm(-self.lift_motor_pwm)
            self.lift_direction = 0  # Reset direction after processing
        else:
            # Stop lift motor if no direction is given
            self.publish_lift_motor_pwm(0)

    def publish_lift_motor_pwm(self, pwm: int):
        """Publish lift motor PWM value"""
        msg = Int32()
        msg.data = pwm
        self.lift_motor_pwm_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    lift_motor_control_node = LiftMotorControlNode()

    rclpy.spin(lift_motor_control_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
