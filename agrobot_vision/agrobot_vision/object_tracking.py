#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32
from agrobot_interfaces.msg import YoloResults, SensorDatas
from simple_pid import PID
from numpy import clip


class ObjectTrackingNode(Node):
    def __init__(self):
        super().__init__('object_tracking')

        # Declare parameters for desired values and PID coefficients
        self.declare_parameter('desired_contour_area', 22.7)
        self.declare_parameter('desired_x_difference', 0)
        self.declare_parameter('desired_z_difference', 0)

        self.declare_parameter('kp_area', 0.015)
        self.declare_parameter('ki_area', 0.0)
        self.declare_parameter('kd_area', 0.0)

        self.declare_parameter('kp_x', 0.05)
        self.declare_parameter('ki_x', 0.000)
        self.declare_parameter('kd_x', 0.00)

        self.declare_parameter('kp_z', 0.1)
        self.declare_parameter('ki_z', 0.0001)
        self.declare_parameter('kd_z', 0.001)
        self.declare_parameter('threshold_area', 1.0)
        self.declare_parameter('threshold_x', 0.3)
        self.declare_parameter('threshold_z', 10)
        self.declare_parameter('target_class_id', 0)  # Class ID to track
        # Angular z velocity for sweeping
        self.declare_parameter('sweep_angular_z', 0.3)
        # Maximum linear velocity (m/s
        self.declare_parameter('max_velocity', 0.35)

        # Get parameters
        self.desired_contour_area = self.get_parameter(
            'desired_contour_area').get_parameter_value().double_value
        self.desired_x_difference = self.get_parameter(
            'desired_x_difference').get_parameter_value().integer_value
        self.desired_z_difference = self.get_parameter(
            'desired_z_difference').get_parameter_value().integer_value

        kp_area = self.get_parameter(
            'kp_area').get_parameter_value().double_value
        ki_area = self.get_parameter(
            'ki_area').get_parameter_value().double_value
        kd_area = self.get_parameter(
            'kd_area').get_parameter_value().double_value

        kp_x = self.get_parameter('kp_x').get_parameter_value().double_value
        ki_x = self.get_parameter('ki_x').get_parameter_value().double_value
        kd_x = self.get_parameter('kd_x').get_parameter_value().double_value

        kp_z = self.get_parameter('kp_z').get_parameter_value().double_value
        ki_z = self.get_parameter('ki_z').get_parameter_value().double_value
        kd_z = self.get_parameter('kd_z').get_parameter_value().double_value

        self.threshold_area = self.get_parameter(
            'threshold_area').get_parameter_value().double_value
        self.threshold_x = self.get_parameter(
            'threshold_x').get_parameter_value().double_value
        self.threshold_z = self.get_parameter(
            'threshold_z').get_parameter_value().integer_value

        self.target_class_id = self.get_parameter(
            'target_class_id').get_parameter_value().integer_value
        self.sweep_angular_z = self.get_parameter(
            'sweep_angular_z').get_parameter_value().double_value
        self.max_velocity = self.get_parameter(
            'max_velocity').get_parameter_value().double_value

        # Initialize PID controllers
        self.pid_area = PID(kp_area, ki_area, kd_area, setpoint=0)
        self.pid_x = PID(kp_x, ki_x, kd_x, setpoint=self.desired_x_difference)
        self.pid_z = PID(kp_z, ki_z, kd_z, setpoint=self.desired_z_difference)

        self.linearx = 0.0
        self.lineary = 0.0
        self.angularz = 0.0

        # Subscribe to YOLO results topic
        self.subscription = self.create_subscription(
            YoloResults,
            'yolo_results',
            self.yolo_callback,
            10
        )

        self.start_sub = self.create_subscription(
            Bool,
            'start_object_tracking',
            self.tracking_callback,
            10
        )

        self.tracking_subscription = self.create_subscription(
            SensorDatas,
            'sensor_data',
            self.sensor_data_callback,
            10
        )
        # Publisher
        self.publisher = self.create_publisher(
            Twist, 'cmd_vel', 10)
        self.lift_command_publisher = self.create_publisher(
            Int32, 'lift_direction', 10)
        self.completed_publisher = self.create_publisher(
            Bool, 'object_tracking_completed', 10)

        self.do_tracking = False
        self.ultrasonic_1 = 0.0
        self.ultrasonic_2 = 0.0
        self.ultrasonic_distance = 0.0

        self.create_timer(0.1, self.yolo_callback)

        self.get_logger().info("Object Tracking Node Initialized.")

    def tracking_callback(self, msg: Bool):
        self.do_tracking = msg.data
        self.get_logger().info("RECEIVED TRACKING INFO")

    def sensor_data_callback(self, msg: SensorDatas):
        self.ultrasonic_1 = msg.ultrasonic_1
        self.ultrasonic_2 = msg.ultrasonic_2
        self.ultrasonic_distance = (self.ultrasonic_1 + self.ultrasonic_2) / 2

    def yolo_callback(self):

        # target_indices = [i for i, class_id in enumerate(
        #     msg.class_ids) if class_id == self.target_class_id]

        if not self.do_tracking:
            # . If tracking is not enabled, do nothing
            self.get_logger().error(
                f"do_tracking:{self.do_tracking}", throttle_duration_sec=1.0)
            self.publish_completed(False)
            return

        # if not target_indices:
        #     # If no target is detected, sweep in angular z
        #     self.sweep()
        #     self.publish_completed(False)
        #     return

        # Sort indices based on contour area in descending order
        # target_indices.sort(key=lambda i: msg.contour_area[i], reverse=True)

        # Select the object with the largest contour area
        # target_index = target_indices[0]

        # Extract contour area and differences for the target object
        # contour_area = msg.contour_area[target_index]
        contour_area = self.ultrasonic_distance
        x_difference = self.ultrasonic_1 - self.ultrasonic_2

        # x_difference = msg.x_differences[target_index]
        # z_difference = msg.z_differences[target_index]

        # self.get_logger().error(f"Contour Area: {contour_area}, X Difference: {x_difference}, Z Difference: {z_difference}")

        # Calculate errors
        error_area = (contour_area - self.desired_contour_area)
        error_x = x_difference - self.desired_x_difference
        # error_z = self.desired_z_difference - z_difference

        self.get_logger().error(f"Errors: Area: {error_area} \n"
                                f"X: {error_x}, ",
                                # "\n Z: {error_z}",
                                throttle_duration_sec=1.0
                                )

        # Compute control signals using PID controllers
        control_area = -self.pid_area(error_area)
        control_x = self.pid_x(error_x)
        # control_z = self.pid_z(error_z)

        # id_target = msg.tracking_id[target_index]

        # self.get_logger().info(
        #     f"\033[93m\nTarget ID: {id_target}\033[0m",
        #     throttle_duration_sec=1.0
        # )

        self.get_logger().info(
            f"\033[91m \nControls: Area: {control_area}\033[0m, "
            f"\033[92m\n X:{control_x}\033[0m, ",
            # f"\033[94m\n Z:{control_z}\033[0m",
            throttle_duration_sec=1.0
        )

        # Check if all conditions are within thresholds
        within_threshold_area = abs(error_area) <= self.threshold_area
        within_threshold_x = abs(error_x) <= self.threshold_x
        # within_threshold_z = abs(error_z) <= self.threshold_z

        # self.get_logger().info(f"Within Thresholds: Area: {within_threshold_area}, X: {within_threshold_x}, Z: {within_threshold_z}")

        # if within_threshold_z:
        #     # Publish lift direction
        #     self.publish_lift_direction(0.0)
        # else:
        #     # Publish lift direction based on control z
        #     self.publish_lift_direction(control_z)

        # if within_threshold_area and within_threshold_x:
        # if within_threshold_area and within_threshold_x:
        print(within_threshold_x)
        if within_threshold_area and within_threshold_x:

            # Publish zero velocities if all conditions are within thresholds
            self.linearx = 0.0
            self.lineary = 0.0
            self.angularz = 0.0
            self.publish_completed(True)
            self.get_logger().info(
                "\033[93mAll conditions met, Stopping the robot\033[0m",
                throttle_duration_sec=1.5)
        else:

            # Apply control signals to Twist message, with threshold checks
            self.linearx = control_area if not within_threshold_area else 0.0
            self.lineary = 0.0  # Assuming no lateral movement control needed
            self.angularz = control_x if not within_threshold_x else 0.0

            self.publish_completed(False)
            self.get_logger().info(
                "\033[92mPublishing control signals...\033[0m", throttle_duration_sec=2.5)

        # Publish the Twist Message
        self.publish_twist(self.linearx, self.lineary, self.angularz)

    def publish_twist(self, linearx, lineary, angularz):
        twist_msg = Twist()
        twist_msg.linear.x = clip(
            linearx, -self.max_velocity, self.max_velocity)
        twist_msg.linear.y = clip(
            lineary, -self.max_velocity, self.max_velocity)
        twist_msg.angular.z = clip(
            angularz, -1.5, 1.5)
        self.publisher.publish(twist_msg)

    def sweep(self):
        # Create Twist message for sweeping in angular z
        self.linearx = 0.0
        self.lineary = 0.0
        self.linearz = 0.0
        # self.angularz = self.sweep_angular_z
        self.angularz = 0.0
        self.publish_twist(self.linearx, self.lineary, self.angularz)

        # Publish sweep command
        self.get_logger().info("Sweeping to find target object...", throttle_duration_sec=2.5)

    def publish_lift_direction(self, value: float):
        # publish 1 if control z is positive, -1 if control z is negative, 0 if control z is zero
        msg = Int32()
        msg.data = 1 if value > 0 else -1 if value < 0 else 0
        self.lift_command_publisher.publish(msg)

    def publish_completed(self, value: bool):
        msg = Bool()
        msg.data = value
        self.completed_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    object_tracking = ObjectTrackingNode()
    rclpy.spin(object_tracking)
    object_tracking.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
