#!/usr/bin/env python3


from construct import max_
import rclpy
import numpy as np
from math import pi
from simple_pid import PID
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from agrobot_interfaces.msg import MotorPWMs, EncoderPulses, WheelAngularVel, PIDWheelError

MIN_PWM = 0
MAX_PWM = 100


class WheelSpeedControl(Node):

    def __init__(self):
        super().__init__("wheel_speed_control")

        # Create Parameter
        self.declare_parameter("debug", False)
        self.declare_parameter("kp", 0.8)  # Reduced Kp significantly
        self.declare_parameter("ki", 0.001)  # Very small Ki
        self.declare_parameter("kd", 0.05)  # Moderate Kd

        self.debug = self.get_parameter("debug").value

        self.enc_sub_ = self.create_subscription(
            EncoderPulses, "encoder_pulses", self.encoder_pulses_callback, 10)

        self.motor_pwm_pub_ = self.create_publisher(
            MotorPWMs, "motor_pwm", 10)

        self.wheel_speed_sub_ = self.create_subscription(
            WheelAngularVel,
            "wheel_angular_vel",
            self.wheel_speed_callback,
            10
        )

        self.error_pub_ = self.create_publisher(
            PIDWheelError, "wheel_speed_errors", 10)

        self.encoder_counts_: list[int] = [0, 0, 0, 0]
        self.previous_encoder_counts_in_loop_: list[int] = [0, 0, 0, 0]
        self.wheel_speed_: list[float] = [0.0, 0.0, 0.0, 0.0]
        self.actual_wheel_speed_: list[float] = [0.0, 0.0, 0.0, 0.0]
        self.errors_: list[float] = [0.0, 0.0, 0.0, 0.0]
        self.integral_terms_: list[float] = [0.0, 0.0, 0.0, 0.0]
        self.previous_errors_: list[float] = [0.0, 0.0, 0.0, 0.0]
        self.control_d: list[float] = [0.0, 0.0, 0.0, 0.0]
        self.alpha = 0.1

        self.encoder_ticks_per_rev = 1464
        self.wheel_radius = 0.05  # m - 100mm Dia
        self.control_frequency = 20  # Hz

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
                self.previous_encoder_counts_in_loop_[i]
            self.actual_wheel_speed_[i] = (
                delta_ticks / self.encoder_ticks_per_rev) * (2 * pi) / dt

            self.get_logger().warn(
                f"Wheel {i+1} ACTUAL Speed: {self.actual_wheel_speed_[i]}"
            )

    def control_loop(self):
        dt = 1/self.control_frequency

        self.calculate_wheel_speeds(dt)
        self.previous_encoder_counts_in_loop_ = self.encoder_counts_.copy()

        kp = self.get_parameter("kp").value
        ki = self.get_parameter("ki").value
        kd = self.get_parameter("kd").value

        motor_pwms = [0] * 4

        for i in range(4):
            # Calculate Error based on absolute target speed
            # Absolute target speed
            target_speed_abs = abs(self.wheel_speed_[i])
            actual_speed_abs = abs(self.actual_wheel_speed_[i])

            self.errors_[i] = target_speed_abs - actual_speed_abs

            # Compute the PID Components
            self.integral_terms_[i] += self.errors_[i] * dt
            delta_error = self.errors_[i] - self.previous_errors_[i]

            # Low pass filter for differential control
            self.control_d[i] = (self.control_d[i] *
                                 (1.0 - self.alpha) + delta_error * self.alpha)

            # Clip the integral error
            self.integral_terms_[i] = np.clip(
                self.integral_terms_[i], -MAX_PWM, MAX_PWM)

            # Calculate proportional error
            proportional_error = kp * self.errors_[i]

            # Calculate the PID output
            pid_output = proportional_error + \
                (ki * self.integral_terms_[i]) + (kd * self.control_d[i])

            # Apply sign based on target speed
            if self.wheel_speed_[i] < 0:
                motor_pwms[i] = -np.clip(int(pid_output), MIN_PWM, MAX_PWM)
            else:
                motor_pwms[i] = np.clip(int(pid_output), MIN_PWM, MAX_PWM)

            # Update the previous errors
            self.previous_errors_[i] = self.errors_[i]

            self.get_logger().error(
                f"Wheel {i+1} Target: {self.wheel_speed_[i]}, Actual: {self.actual_wheel_speed_[i]}, P: {proportional_error}, I: {self.integral_terms_[i]}, D: {self.control_d[i]}, Output: {motor_pwms[i]}"
            )

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
