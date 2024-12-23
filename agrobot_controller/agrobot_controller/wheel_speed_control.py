#!/usr/bin/env python3


import rclpy
import numpy as np
from math import pi
from simple_pid import PID
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from agrobot_interfaces.msg import MotorPWMs, EncoderPulses, WheelAngularVel, PIDWheelError

MIN_PWM = 0
MAX_PWM = 100
MAX_RPM = 112  # Maximum motor RPM
MAX_ANGULAR_VEL = 11.728613  # rad/s equivalent to 112 RPM


class WheelSpeedControl(Node):

    def __init__(self):
        super().__init__("wheel_speed_control")

        # Create Parameter
        self.declare_parameter("debug", False)
        self.declare_parameter("kp", 0.0)  # Reduced Kp significantly
        self.declare_parameter("ki", 0.0)  # Very small Ki
        self.declare_parameter("kd", 0.0)  # Moderate Kd

        self.debug = self.get_parameter("debug").value

        # . Subscriptions
        self.enc_sub_ = self.create_subscription(
            EncoderPulses, "encoder_pulses", self.encoder_pulses_callback, 10)
        self.wheel_speed_sub_ = self.create_subscription(
            WheelAngularVel, "wheel_angular_vel", self.wheel_speed_callback, 10)

        # . Publishers
        self.motor_pwm_pub_ = self.create_publisher(MotorPWMs, "motor_pwm", 10)
        self.error_pub_ = self.create_publisher(
            PIDWheelError, "wheel_speed_errors", 10)

        # . Variables
        self.encoder_counts_ = [0] * 4
        self.previous_encoder_counts_ = [0] * 4
        self.actual_wheel_speed_ = [0.0] * 4
        self.wheel_speed_ = [0.0] * 4
        self.errors_ = [0.0] * 4
        self.previous_errors_ = [0.0] * 4
        self.integral_terms_ = [0.0] * 4
        self.encoder_ticks_per_rev = 1464
        self.control_frequency = 30  # Hz

        # Timer For PID Control
        self.create_timer(1 / self.control_frequency, self.control_loop)

    def wheel_speed_callback(self, msg: WheelAngularVel):

        self.wheel_speed_ = [
            msg.wheel_1_angular_vel,
            msg.wheel_2_angular_vel,
            msg.wheel_3_angular_vel,
            msg.wheel_4_angular_vel
        ]

        if self.debug:
            self.get_logger().debug(
                f"Wheel Angular Velocities: {self.wheel_speed_[0]}, {self.wheel_speed_[1]}, {self.wheel_speed_[2]}, {self.wheel_speed_[3]}"
            )

    def encoder_pulses_callback(self, msg: EncoderPulses):
        self.encoder_counts_ = [
            msg.encoder_1_pulse,
            msg.encoder_2_pulse,
            msg.encoder_3_pulse,
            msg.encoder_4_pulse
        ]

        if self.debug:
            self.get_logger().debug(
                f"Encoder Pulses: {self.encoder_counts_[0]}, {self.encoder_counts_[1]}, {self.encoder_counts_[2]}, {self.encoder_counts_[3]}"
            )

    def calculate_wheel_speeds(self, dt: float):
        for i in range(4):
            delta_ticks = self.encoder_counts_[i] - \
                self.previous_encoder_counts_[i]
            self.actual_wheel_speed_[i] = (
                delta_ticks / self.encoder_ticks_per_rev) * (2 * pi) / dt

            self.get_logger().warn(
                f"Wheel {i+1} ACTUAL Speed: {self.actual_wheel_speed_[i]}"
            )

    def control_loop(self):
        dt = 1/self.control_frequency

        self.calculate_wheel_speeds(dt)

        for i in range(4):
            self.get_logger().warn(
                f"Wheel {i+1} , Actual: {self.actual_wheel_speed_[i]}"
            )
        self.previous_encoder_counts_ = self.encoder_counts_.copy()

        kp = self.get_parameter("kp").value
        ki = self.get_parameter("ki").value
        kd = self.get_parameter("kd").value

        motor_pwms = [0] * 4

        for i in range(4):
            # Motor Angular Vel 0
            if self.wheel_speed_[i] == 0:
                self.integral_terms_[i] = 0.0
                self.errors_[i] = 0.0
                self.previous_errors_[i] = 0.0
                motor_pwms[i] = 0
                continue

            # Calculate base PWM using linear scaling
            abs_target_speed = abs(self.wheel_speed_[i])
            base_pwm = (abs_target_speed / MAX_ANGULAR_VEL) * MAX_PWM

            # Calculate PID error correction
            error = abs(self.wheel_speed_[i]) - \
                abs(self.actual_wheel_speed_[i])
            self.errors_[i] = error
            self.integral_terms_[i] += error * dt
            derivative = (error - self.previous_errors_[i]) / dt

            pid_correction = (kp * error) + \
                (ki * self.integral_terms_[i]) + (kd * derivative)

            # Calculate final PWM
            pwm_output = base_pwm + pid_correction
            pwm_output = np.clip(pwm_output, MIN_PWM, MAX_PWM)

            # Apply Direction
            motor_pwms[i] = pwm_output if self.wheel_speed_[
                i] > 0 else -pwm_output
            self.previous_errors_[i] = error

        self.publish_motor_pwm(motor_pwms)
        self.publish_errors()

    def publish_motor_pwm(self, motor_pwms: list[int]):
        motor_pwm_msg = MotorPWMs()
        motor_pwm_msg.motor1pwm = int(motor_pwms[0])
        motor_pwm_msg.motor2pwm = int(motor_pwms[1])
        motor_pwm_msg.motor3pwm = int(motor_pwms[2])
        motor_pwm_msg.motor4pwm = int(motor_pwms[3])

        self.motor_pwm_pub_.publish(motor_pwm_msg)

        if self.debug:
            self.get_logger().info(
                f"Motor PWMs: {motor_pwms[0]}, {motor_pwms[1]}, {motor_pwms[2]}, {motor_pwms[3]}"
            )

    def publish_errors(self):
        error_msg = PIDWheelError()
        error_msg.wheel_1_error = self.errors_[0]
        error_msg.wheel_2_error = self.errors_[1]
        error_msg.wheel_3_error = self.errors_[2]
        error_msg.wheel_4_error = self.errors_[3]

        self.error_pub_.publish(error_msg)

        if self.debug:
            self.get_logger().info(
                f"PID Errors: {self.errors_[0]}, {self.errors_[1]}, {self.errors_[2]}, {self.errors_[3]}"
            )


def main(args=None):
    rclpy.init(args=args)

    wheel_speed_control = WheelSpeedControl()

    try:
        rclpy.spin(wheel_speed_control)
    except KeyboardInterrupt:
        pass

    wheel_speed_control.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
