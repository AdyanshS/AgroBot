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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from agrobot_interfaces.msg import WheelAngularVel
from geometry_msgs.msg import Twist

# 1/Root2 Value
ONE_BY_ROOT2 = 0.70710678118
ROOT2 = 1.41421356237
MAX_WHEEL_ANG_VEL = 11.7286  # rad/s
MAX_LIN_X = 0.8293  # m/s
MAX_LIN_Y = 0.8293  # m/s
MAX_ANG_Z = 2.5299  # rad/s


def euler2quat(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion.
    Parameters
    ----------
    roll : float
        Rotation angle around the x-axis (in radians).
    pitch : float
        Rotation angle around the y-axis (in radians).
    yaw : float
        Rotation angle around the z-axis (in radians).
    Returns
    -------
    q : list
        Quaternion [w, x, y, z]
    """
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [w, x, y, z]


class XDriveController(Node):

    def __init__(self):
        super().__init__("x_drive_controller_py")

        # Create Parameter
        self.declare_parameter("debug", False)
        self.declare_parameter("publish_tf", True)

        self.debug = self.get_parameter("debug").value
        self.publish_tf = self.get_parameter("publish_tf").value

        self.wheel_ang_sub = self.create_subscription(
            WheelAngularVel, "wheel_angular_vel/feedback", self.ang_vel_feedback_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel/drive", self.cmd_vel_callback, 10)

        self.wheel_ang_pub = self.create_publisher(
            WheelAngularVel, "wheel_angular_vel/control", 10)
        self.odom_pub = self.create_publisher(
            Odometry, "odom", 10)

        self.current_wheel_ang_vel: list[float] = [0.0] * 4

        self.wheel_radius = 0.05  # meters
        self.robot_radius = 0.2318  # meters

        # State Variable for pose Tracking
        self.x = 0.0      # Robot's x-coordinate (m)
        self.y = 0.0      # Robot's y-coordinate (m)
        self.theta = 0.0  # Robot's orientation (rad)

        self.last_time = self.get_clock().now()

        # Create a Transform Broadcaster to publish the TF
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

    def cmd_vel_callback(self, msg: Twist):
        """ Callback function for the cmd_vel topic. """
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
        """
        Calculate the wheel angular velocities for a robot given the desired velocities in the x, y, and rotational directions.
        This function uses the inverse kinematics of a robot with an X-drive configuration to compute the angular velocities
        of the wheels based on the desired linear and angular velocities of the robot.
        Args:
            x_dot (float): Desired velocity in the x-direction (m/s).
            y_dot (float): Desired velocity in the y-direction (m/s).
            w_dot (float): Desired angular velocity around the z-axis (rad/s).
        Returns:
            list: A list of angular velocities for each wheel (rad/s).
        The transformation matrix T is defined as:
            T = | -r  1/√2  -1/√2 |
                | -r  1/√2   1/√2 |
                | -r -1/√2   1/√2 |
                | -r -1/√2  -1/√2 |
        where r is the robot radius.
        The wheel angular velocities are calculated as:
            wheel_ang_vel = T * [w_dot, x_dot, y_dot]^T / wheel_radius
        """

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
        """
        Calculate the robot's forward kinematics based on the angular velocities of the wheels.
        Args:
            wheel_ang_vel (list[float]): A list of four floats representing the angular velocities of the wheels.
        Returns:
            tuple: A tuple of three floats representing the robot's linear and angular velocities (w_dot, x_dot y_dot).
        The pseudo-inverse transformation matrix T_inv is defined as:
            T_inv = | -1/R  -1/R  -1/R  -1/R |
                    |  √2    √2   -√2   -√2  |
                    | -√2    √2    √2   -√2  |
            where r is the robot radius.
        The robot velocities are calculated as:
            robot_vel = (T_inv * wheel_ang_vel) * wheel_radius / 4
        """

        # Pseudo Inverse Transformation Matrix
        T_inv = np.array([
            [-1/self.robot_radius, -1/self.robot_radius, -
                1/self.robot_radius, -1/self.robot_radius],
            [ROOT2, ROOT2, -ROOT2, -ROOT2],
            [-ROOT2, ROOT2, ROOT2, -ROOT2]
        ])

        # Calculate Robot Velocities
        robot_vel = np.dot(T_inv, wheel_ang_vel) * self.wheel_radius / 4

        robot_vel_list = list(robot_vel)

        return robot_vel_list[0], robot_vel_list[1], robot_vel_list[2]

    def ang_vel_feedback_callback(self, msg: WheelAngularVel):
        """ Callback function for the wheel angular velocity feedback. """

        self.current_wheel_ang_vel[0] = msg.wheel_1_angular_vel
        self.current_wheel_ang_vel[1] = msg.wheel_2_angular_vel
        self.current_wheel_ang_vel[2] = msg.wheel_3_angular_vel
        self.current_wheel_ang_vel[3] = msg.wheel_4_angular_vel

        # * Compute Robot Velocities using Forward Kinematics
        w_dot, x_dot, y_dot = self.forward_kinematics(
            self.current_wheel_ang_vel)

        # * Get the current time and calculate the `dt` (time difference)
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Update pose using the velocities
        self.x += x_dot * cos(self.theta) * dt - y_dot * sin(self.theta) * dt
        self.y += x_dot * sin(self.theta) * dt + y_dot * cos(self.theta) * dt
        self.theta += w_dot * dt

        # Normalize theta to the range [-pi, pi]
        self.theta = (self.theta + pi) % (2 * pi) - pi

        # Publish the odometry message
        self.publish_odometry(self.x, self.y, self.theta, x_dot, y_dot, w_dot)

        # Publish the transform between the odom and base_link frames
        if self.publish_tf:
            self.publish_odom_tf(self.x, self.y, self.theta)

        if self.debug:
            self.get_logger().info(
                f"Robot Velocities: {x_dot}, {y_dot}, {w_dot}"
            )

        if self.debug:
            self.get_logger().info(
                f"Current_wheel_ang_vel: {self.current_wheel_ang_vel[0]}, {self.current_wheel_ang_vel[1]}, {self.current_wheel_ang_vel[2]}, {self.current_wheel_ang_vel[3]}"
            )

    def publish_odometry(self, x: float, y: float, theta: float, x_dot: float, y_dot: float, w_dot: float):
        """ Publish the odometry message. """
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"

        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0

        q = euler2quat(0.0, 0.0, theta)
        odom_msg.pose.pose.orientation.w = q[0]
        odom_msg.pose.pose.orientation.x = q[1]
        odom_msg.pose.pose.orientation.y = q[2]
        odom_msg.pose.pose.orientation.z = q[3]

        odom_msg.twist.twist.linear.x = x_dot
        odom_msg.twist.twist.linear.y = y_dot
        odom_msg.twist.twist.angular.z = w_dot

        self.odom_pub.publish(odom_msg)

    def publish_odom_tf(self, x: float, y: float, theta: float):
        """ Publish the transform between the odom and base_link frames. """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        q = euler2quat(0.0, 0.0, theta)
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]

        self.tf_broadcaster.sendTransform(t)

    def send_wheel_ang_vel(self, wheel_ang_vel: list[float]):
        """ Publish the wheel angular velocities to the control topic. """
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
