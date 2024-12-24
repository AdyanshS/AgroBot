#!/usr/bin/env python3

"""
X-Drive 4 Wheeled Omni Wheel Robot Controller
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
from agrobot_interfaces.msg import WheelAngularVel
from geometry_msgs.msg import Twist

# 1/Root2 Value
ONE_BY_ROOT2 = 0.70710678118
ROOT2 = 1.41421356237
MAX_WHEEL_ANG_VEL = 11.7286  # rad/s
MAX_LIN_X = 0.8293  # m/s
MAX_LIN_Y = 0.8293  # m/s
MAX_ANG_Z = 2.5299  # rad/s


class XDriveController(Node):

    def __init__(self):
        super().__init__("x_drive_controller_py")

        # Create Parameter
        self.declare_parameter("debug", True)
        self.debug = self.get_parameter("debug").value

        self.wheel_ang_sub = self.create_subscription(
            WheelAngularVel,
            "wheel_angular_vel/feedback",
            self.ang_vel_feedback_callback,
            10
        )

        self.wheel_ang_pub = self.create_publisher(
            WheelAngularVel,
            "wheel_angular_vel/control",
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "cmd_vel/drive",
            self.cmd_vel_callback,
            10
        )

        self.current_wheel_ang_vel: list[float] = [0.0] * 4

        self.wheel_radius = 0.05  # meters
        self.robot_radius = 0.2318  # meters

    def cmd_vel_callback(self, msg: Twist):
        x_dot = msg.linear.x
        y_dot = msg.linear.y
        w_dot = msg.angular.z

        if self.debug:
            self.get_logger().error(
                f"Robot Velocities: {x_dot}, {y_dot}, {w_dot}"
            )
        wheel_ang_vel = self.inverse_kinematics(x_dot, y_dot, w_dot)

        self.send_wheel_ang_vel(wheel_ang_vel)

        if self.debug:
            self.get_logger().info(
                f"Wheel Angular Velocities: {wheel_ang_vel[0]}, {wheel_ang_vel[1]}, {wheel_ang_vel[2]}, {wheel_ang_vel[3]}"
            )

    def inverse_kinematics(self, x_dot: float, y_dot: float, w_dot: float):

        # MAtrix
        robot_vel = np.array([w_dot, x_dot, y_dot])

        # Transformation Matrix
        T = np.array([
            [-self.robot_radius, ONE_BY_ROOT2, -ONE_BY_ROOT2],
            [-self.robot_radius, ONE_BY_ROOT2, ONE_BY_ROOT2],
            [-self.robot_radius, -ONE_BY_ROOT2, ONE_BY_ROOT2],
            [-self.robot_radius, -ONE_BY_ROOT2, -ONE_BY_ROOT2]
        ])

        # Calculate Wheel Angular Velocities
        wheel_ang_vel = np.dot(T, robot_vel) / self.wheel_radius

        return list(wheel_ang_vel)

    def forward_kinematics(self, wheel_ang_vel: list[float]):
        # Pseudo Inverse Transformation Matrix
        T_inv = np.array([
            [-1/self.robot_radius, -1/self.robot_radius,
                -1/self.robot_radius, -1/self.robot_radius],
            [ROOT2, ROOT2, -ROOT2, -ROOT2],
            [-ROOT2, ROOT2, ROOT2, -ROOT2]
        ])

        # Calculate Robot Velocities
        robot_vel = np.dot(T_inv, wheel_ang_vel) * (self.wheel_radius) * (1/4)

        return list(robot_vel)

    def ang_vel_feedback_callback(self, msg: WheelAngularVel):

        self.current_wheel_ang_vel[0] = msg.wheel_1_angular_vel
        self.current_wheel_ang_vel[1] = msg.wheel_2_angular_vel
        self.current_wheel_ang_vel[2] = msg.wheel_3_angular_vel
        self.current_wheel_ang_vel[3] = msg.wheel_4_angular_vel

        robot_vel = self.forward_kinematics(self.current_wheel_ang_vel)

        if self.debug:
            self.get_logger().info(
                f"Robot Velocities: {robot_vel[0]}, {robot_vel[1]}, {robot_vel[2]}"
            )

        if self.debug:
            self.get_logger().info(
                f"Current_wheel_ang_vel: {self.current_wheel_ang_vel[0]}, {self.current_wheel_ang_vel[1]}, {self.current_wheel_ang_vel[2]}, {self.current_wheel_ang_vel[3]}"
            )

    def send_wheel_ang_vel(self, wheel_ang_vel: list[float]):
        motor_pwm_msg = WheelAngularVel()
        motor_pwm_msg.wheel_1_angular_vel = wheel_ang_vel[0]
        motor_pwm_msg.wheel_2_angular_vel = wheel_ang_vel[1]
        motor_pwm_msg.wheel_3_angular_vel = wheel_ang_vel[2]
        motor_pwm_msg.wheel_4_angular_vel = wheel_ang_vel[3]

        self.wheel_ang_pub.publish(motor_pwm_msg)

        if self.debug:
            self.get_logger().info(
                f"Motor PWMs: {wheel_ang_vel[0]}, {wheel_ang_vel[1]}, {wheel_ang_vel[2]}, {wheel_ang_vel[3]}"
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
