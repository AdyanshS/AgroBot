#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import enum

from std_msgs.msg import Float32, Int32, Bool
from geometry_msgs.msg import Twist


class RobotStates(enum.Enum):
    """ Enum to represent different states of the robot"""
    INIT = 0
    START_TRACKING = 1
    TRACKING = 2
    POST_TRACKING = 3


class CottonPlucking(Node):

    def __init__(self):
        super().__init__("cotton_plucking")

        # . Create publishers
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

        # . Create subscribers
        self.completed_tracking_subscriber = self.create_subscription(
            Bool, 'object_tracking_completed', self.completed_tracking_callback, 10)

        # . Internal Variables
        self.object_tracking_completed = False
        self.current_state = RobotStates.INIT  # Start in INIT state

        # . Sequence Management
        self.sequence_steps = []
        self.sequence_index = 0

        # . Timers
        self.main_timer = self.create_timer(0.1, self.process_state)
        self.delay_timer = None
        self.delay_done = False

        self.get_logger().info(
            "\033[92mCottonPlucking node initialized and state machine started.\033[0m")

    # . ----------------- State Machine Functions -----------------

    def process_state(self):
        """ Main state machine logic periodically called by main_timer"""
        if self.current_state == RobotStates.INIT:
            self.get_logger().info("\033[92mEntered State: INIT\033[0m")
            # ? 1. Retract the arm, open the gripper, claw front facing
            self.SetArmRetracted()
            self.SetGripOpen()
            self.SetClawFrontFacing()

            # Transition
            self.current_state = RobotStates.START_TRACKING

        elif self.current_state == RobotStates.START_TRACKING:
            self.get_logger().info(
                "\033[92mEntered State: START_TRACKING\033[0m")
            # ? 2. Start object tracking
            self.object_tracking_completed = False  # Reset flag here
            self.start_object_tracking()
            self.start_object_tracking()
            self.start_object_tracking()
            self.start_object_tracking()

            # Transition
            self.current_state = RobotStates.TRACKING

        elif self.current_state == RobotStates.TRACKING:
            self.get_logger().info(
                "\033[92mEntered State: TRACKING\033[0m", once=True)

            self.get_logger().info(
                f"\033[92mTracking object...:{self.object_tracking_completed}\033[0m", throttle_duration_sec=2.0)

            # ? 3. Wait 1until object tracking is completed
            if self.object_tracking_completed:
                self.stop_object_tracking()  # Stop object tracking
                self.SetCmdVel(0.0, 0.0, 0.0)  # Stop the robot

                # Prepare the post-tracking sequence
                self.current_state = RobotStates.POST_TRACKING
                self.prepare_post_tracking_sequence()

        elif self.current_state == RobotStates.POST_TRACKING:
            self.get_logger().info(
                "\033[92mEntered State: POST_TRACKING\033[0m", once=True)
            # ? Steps after tracking are managed by sequence steps and delay timer
            pass

    def prepare_post_tracking_sequence(self):
        """ Prepare a list of (function, delay) pairs for each step.
        Then start executing them via schedule_next_sequence_step.
        """
        self.sequence_steps = [
            (self.SetArmExtended, 2.0),
            (self.SetGripClose, 1.0),
            (self.SetClawBackFacing, 1.0),
            (self.SetArmRetracted, 2.0),
            (self.SetGripOpen, 1.0),
            (self.SetGripClose, 1.0),
            (self.SetClawFrontFacing, 1.0),
            (self.SetGripOpen, 1.0),
            (self.move_robot_backwards, 2.3),
            (self.stop_robot, 1.0),
            (self.SetArmExtended, 2.0),
            (self.SetGripClose, 1.0),
            (self.SetClawBackFacing, 1.0),
            (self.SetArmRetracted, 2.0),
            (self.SetGripOpen, 1.0),
            (self.SetGripClose, 1.0),
            (self.SetClawFrontFacing, 1.0),
            (self.move_robot_backwards, 2.0),
            (self.stop_robot, 1.0),

            # (self.restart_tracking_cycle, 0.0)
        ]

        self.sequence_index = 0
        self.schedule_next_sequence_step()

    def schedule_next_sequence_step(self):
        """ 
        Executes the next action in sequence_steps and sets a non-blocking delay
        for the next step. Only one timer is used at a time to avoid conflicts.
        """
        if self.sequence_index < len(self.sequence_steps):

            action, delay = self.sequence_steps[self.sequence_index]
            self.sequence_index += 1

            # * Perform the action immediately
            action()

            # * If theres'a delay -> Set a non-blocking timer for next step
            if delay > 0:
                self.set_delay(delay)
            else:
                # If no delay -> Schedule the next step immediately
                self.schedule_next_sequence_step()

        else:
            # ? Sequence completed
            self.get_logger().info(
                "\033[92mPost-tracking sequence completed.\033[0m")
            pass

    def set_delay(self, duration):
        """ Non-blocking delay for specific state transitions """
        self.delay_done = False

        #! Cancel any existing timer to avoid duplicates
        if self.delay_timer is not None:
            self.delay_timer.cancel()

        self.delay_timer = self.create_timer(duration, self.delay_callback)
        # self.get_logger().info(
        #     f"\033[93mDelay timer Started: {self.delay_timer.is_ready()}\033[0m")

    def delay_callback(self):
        """ Callback to mark the end of the delay, then move to next step """
        self.delay_done = True
        self.delay_timer.destroy()
        # self.get_logger().info(
        #     f"\033[93mDelay timer canceled: {self.delay_timer.is_ready()}\033[0m")
        self.delay_timer = None

        # * Once the delay is done -> Schedule the next step
        self.schedule_next_sequence_step()

    def restart_tracking_cycle(self):
        """
        Reset the object tracking and got to START_TRACKING
        to begin the cycle again if desired
        """
        self.object_tracking_completed = False  # Critical reset
        self.current_state = RobotStates.START_TRACKING
        self.get_logger().info(
            "\033[92mRestarting tracking cycle...\033[0m")

    # . ----------------- Control Functions --------------------------
    # ----------------- Claw Servo Control Functions -----------------

    def SetClawGroundFacing(self):
        """ Set the claw to ground facing position """
        msg = Float32()
        msg.data = 180.0
        self.claw_publisher.publish(msg)
        self.get_logger().info(
            "\033[33mClaw set to ground facing position.\033[0m")

    def SetClawFrontFacing(self):
        """ Set the claw to front facing position """
        msg = Float32()
        msg.data = 260.0
        self.claw_publisher.publish(msg)
        self.get_logger().info(
            "\033[33mClaw set to front facing position.\033[0m")

    def SetClawBackFacing(self):
        """ Set the claw to back facing position """
        msg = Float32()
        msg.data = 115.0
        self.claw_publisher.publish(msg)
        self.get_logger().info(
            "\033[33mClaw set to back facing position.\033[0m")

    # ----------------- Grip Servo Control Functions -----------------
    def SetGripOpen(self):
        """ Set the grip to open position """
        msg = Float32()
        msg.data = 180.0
        self.grip_publisher.publish(msg)
        self.get_logger().info("\033[34mGrip set to open position.\033[0m")

    def SetGripClose(self):
        """ Set the grip to close position """
        msg = Float32()
        msg.data = 145.0
        self.grip_publisher.publish(msg)
        self.get_logger().info("\033[34mGrip set to close position.\033[0m")

    # ----------------- Arm Servo Control Functions -----------------
    def SetArmExtended(self):
        """ Set the arm to extended position """
        msg = Float32()
        msg.data = 300.0
        self.arm_publisher.publish(msg)
        self.get_logger().info("\033[36mArm set to extended position.\033[0m")

    def SetArmRetracted(self):
        """ Set the arm to retracted position """
        msg = Float32()
        msg.data = 180.0
        self.arm_publisher.publish(msg)
        self.get_logger().info("\033[36mArm set to retracted position.\033[0m")

    # ----------------- Lift Control Functions -----------------
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

    # ----------------- YOLO Control Functions -----------------
    def start_object_tracking(self):
        """ Start object tracking """
        msg = Bool()
        msg.data = True
        self.object_tracking_publisher.publish(msg)
        self.get_logger().info("\033[37mObject tracking started.\033[0m")

    def stop_object_tracking(self):
        """ Stop object tracking """
        msg = Bool()
        msg.data = False
        self.object_tracking_publisher.publish(msg)
        self.get_logger().info("\033[37mObject tracking Stopped.\033[0m")

    def completed_tracking_callback(self, msg: Bool):
        """ Callback function for object tracking completed """
        self.object_tracking_completed = msg.data
        self.get_logger().info(
            f"\033[37mObject tracking completed msg received: {self.object_tracking_completed}\033[0m", once=True)

    # ----------------- Robot Vel Control Functions -----------------
    def SetCmdVel(self, linear_x, linear_y, angular_z):
        """ Set the cmd_vel """
        self.cmd_vel_publish(linear_x, linear_y, angular_z)
        self.get_logger().info(
            f"\033[93mSetting cmd_vel: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}\033[0m")

    def cmd_vel_publish(self, linear_x, linear_y, angular_z):
        """ Publish cmd_vel """
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.linear.y = linear_y
        twist_msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist_msg)

    def move_robot_backwards(self):
        """ Move the robot backwards """
        self.get_logger().info("\033[91mMoving robot backwards...\033[0m")
        self.SetCmdVel(0.0, 0.1, 0.0)

    def stop_robot(self):
        """ Stop the robot """
        self.get_logger().info("\033[91mStopping the robot...\033[0m")
        self.SetCmdVel(0.0, 0.0, 0.0)


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
