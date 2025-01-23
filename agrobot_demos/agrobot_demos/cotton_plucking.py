#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32, Bool
from geometry_msgs.msg import Twist


class CottonPlucking(Node):

    def __init__(self):
        super().__init__("cotton_plucking")

        self.lift_direction_publisher = self.create_publisher(
            Int32, 'lift_direction', 10)
        self.claw_publisher = self.create_publisher(
            Float32, 'servo_angle/claw', 10)
        self.grip_publisher = self.create_publisher(
            Float32, 'servo_angle/grip', 10)
        self.arm_publisher = self.create_publisher(
            Float32, 'servo_angle/arm', 10)
        self.object_tracking_publisher = self.create_publisher(
            Bool, 'start_object_tracking', 10)
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 'cmd_vel', 10)

        # Default values
        self.current_arm_angle = 240.0
        self.current_claw_angle = 180.0

    def grip_open(self):
        """ Open the grip """
        msg = Float32()
        msg.data = 180.0
        self.grip_publisher.publish(msg)

    def grip_close(self):
        """ Close the grip """
        msg = Float32()
        msg.data = 145.0
        self.grip_publisher.publish(msg)

    def claw_angle_increase(self):
        """ Increase the claw angle """
        self.current_claw_angle += 5
        msg = Float32()
        msg.data = self.current_claw_angle
        self.claw_publisher.publish(msg)

    def claw_angle_decrease(self):
        """ Decrease the claw angle """
        self.current_claw_angle -= 5
        msg = Float32()
        msg.data = self.current_claw_angle
        self.claw_publisher.publish(msg)

    def arm_angle_increase(self):
        """ Increase the arm angle """
        self.current_arm_angle += 5
        msg = Float32()
        msg.data = self.current_arm_angle
        self.arm_publisher.publish(msg)

    def arm_angle_decrease(self):
        """ Decrease the arm angle """
        self.current_arm_angle -= 5
        msg = Float32()
        msg.data = self.current_arm_angle
        self.arm_publisher.publish(msg)

    def lift_up(self):
        """ Move the lift up """
        msg = Int32()
        msg.data = 1
        self.lift_direction_publisher.publish(msg)

    def lift_down(self):
        """ Move the lift down """
        msg = Int32()
        msg.data = -1
        self.lift_direction_publisher.publish(msg)

    def cmd_vel_publish(self, linear_x, linear_y, angular_z):
        """ Publish cmd_vel """
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist_msg)


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
