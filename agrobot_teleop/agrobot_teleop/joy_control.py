#!/usr/bin/env python3


import time
import os
import signal
import subprocess

from sympy import N, li

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, Int32
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


# # Constants for button and axis indices
BUTTON_TRIANGLE = 2
BUTTON_CIRCLE = 1
BUTTON_SQUARE = 3
BUTTON_CROSS = 0
AXIS_LEFT_RIGHT = 6  # -1 for right, 1 for left: D-pad left and right
AXIS_UP_DOWN = 7  # -1 for down, 1 for up: D-pad up and down
R1_BUTTON = 5
L1_BUTTON = 4
# PS_BUTTON = 12
# R3_BUTTON = 11
# L3_BUTTON = 10


class JoyControl(Node):
    """Joy Control Node"""

    def __init__(self):
        """Initialize the JoyControl node."""
        super().__init__('joy_control')

        self.joy_subscription = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

        self.lift_direction_publisher = self.create_publisher(
            Int32, 'lift_direction', 10)
        self.claw_publisher = self.create_publisher(
            Float32, 'servo_angle/claw', 10)
        self.grip_publisher = self.create_publisher(
            Float32, 'servo_angle/grip', 10)
        self.arm_publisher = self.create_publisher(
            Float32, 'servo_angle/arm', 10)

        self.prev_buttons = {
            BUTTON_TRIANGLE: 0,
            BUTTON_CIRCLE: 0,
            BUTTON_SQUARE: 0,
            BUTTON_CROSS: 0,
            AXIS_LEFT_RIGHT: 0.0,
            AXIS_UP_DOWN: 0.0
        }  # store the previous button states

        self.press_times = {}            # store the press time for each button
        self.click_counts = {}           # store click counts for multi-clicks
        self.multi_click_timers = {}     # Store timers for multi-click processing
        self.callback_invoked = {}       # Track if a callback has been invoked

        # Process object to store the launched process
        self.process = None

        # Default values
        self.current_arm_angle = 240.0
        self.current_claw_angle = 180.0

        self.get_logger().info('Joy subscriber node has been started')

    def handle_response(self, future):
        """ Handle the response from the service """
        try:
            response = future.result()
            if response is not None:
                for result in response.results:
                    if result.successful:
                        self.get_logger().info(
                            f"Parameter {result.name} set successfully.")
                    else:
                        self.get_logger().warn(
                            f"Failed to set {result.name}: {result.reason}")
            else:
                self.get_logger().error("Service call failed")
        except Exception as e:
            self.get_logger().error(f"Service call error: {str(e)}")

    def handle_button(self, current_value, prev_value, button_name, press_callback, release_callback):
        """
        *Handle the button press and release. \n

        :param current_value: The current state of the button (1 for pressed, 0 for released).
        :type current_value: int
        :param prev_value: The previous state of the button (1 for pressed, 0 for released).
        :type prev_value: int
        :param button_name: The name of the button being handled.
        :type button_name: str
        :param press_callback: The callback function to be called when the button is pressed.
        :type press_callback: function
        :param release_callback: The callback function to be called when the button is released.
        :type release_callback: function
        """
        # Detect rising edge (0 -> 1) for the button
        if current_value == 1 and prev_value == 0:
            press_callback()
            # self.get_logger().info(f'{button_name} pressed')

        # Detect falling edge (1 -> 0) for the button
        if current_value == 0 and prev_value == 1 and release_callback:
            release_callback()
            # self.get_logger().info(f'{button_name} released')

    def handle_button_all(self, current_value, prev_value, button_name, callbacks: dict,
                          long_press_threshold=2.0, multi_click_threshold=0.7):
        """
        *Handle button press, long press and multi-click events.

        :param current_value: The current button state (1=pressed, 0=released).
        :param prev_value: The previous button state.
        :param button_name: Name of the button for logging.
        :param callbacks: Dictionary with callback functions for: \n
            - single_press 
            - long press
            - double_click
            - triple_click
        :param long_press_threshold: Duration threshold for long press detection (default=2.0 seconds).
        :param multi_click_threshold: Duration threshold for multi-click detection (default=0.7 seconds).

        Example:
        ```python
        self.handle_button_all(
            button_triangle,
            self.prev_buttons.get(BUTTON_TRIANGLE, 0),
            'Triangle_button',
            callbacks={
                 'single_press': self.single_press_command,
                 'long_press': self.long_press_command,
                 'double_click': self.double_click_command,
                 'triple_click': self.triple_click_command,
            },
            long_press_threshold=2.0,
            multi_click_threshold=0.7
            )
          ```   
        """
        current_time = time.time()

        # Button Pressed (Rising Edge)
        if current_value == 1 and prev_value == 0:
            self.press_times[button_name] = current_time  # Register press time
            self.click_counts[button_name] = self.click_counts.get(
                button_name, 0) + 1
            # Reset callback invoked flag
            self.callback_invoked[button_name] = False

        # Button Released (Falling Edge)
        if current_value == 0 and prev_value == 1:
            press_duration = current_time - self.press_times[button_name]

            # Long Press Handling
            if press_duration >= long_press_threshold:
                if 'long_press' in callbacks and not self.callback_invoked[button_name]:
                    callbacks['long_press']()  # Trigger long press callback
                    # self.get_logger().info(
                    # f"{button_name} long press detected")
                    self.callback_invoked[button_name] = True
                # Reset click count after long press
                self.click_counts[button_name] = 0

            else:
                # Start multi-click timer if not already started
                if button_name not in self.multi_click_timers:
                    self.multi_click_timers[button_name] = self.create_timer(
                        multi_click_threshold, lambda: self.process_click(
                            button_name, callbacks)
                    )

    def process_click(self, button_name, callbacks):
        """ Handle multi-click actions after multi-click threshold expires. """
        if self.click_counts[button_name] == 1:
            if 'single_press' in callbacks:
                callbacks['single_press']()  # Trigger single press event
                # self.get_logger().info(f"{button_name} single press detected")

        elif self.click_counts[button_name] == 2:
            if 'double_click' in callbacks:
                callbacks['double_click']()  # Trigger double click event
                # self.get_logger().info(f"{button_name} double click detected")

        elif self.click_counts[button_name] == 3:
            if 'triple_click' in callbacks:
                callbacks['triple_click']()  # Trigger triple click event
                # self.get_logger().info(f"{button_name} triple click detected")

        # Reset click count after processing
        self.click_counts[button_name] = 0
        # Destroy the timer once the click is processed
        self.multi_click_timers[button_name].destroy()
        del self.multi_click_timers[button_name]

    def handle_axis(self, current_value, prev_value, axis_name, positive_callback, negative_callback):
        """
        *This method detects the rising edge transitions for the specified axis and 
        *triggers the corresponding callbacks.\n

        :param current_value: The current value of the axis.
        :type current_value: int

        :param prev_value: The previous value of the axis.
        :type prev_value: int

        :param axis_name: The name of the axis being handled.
        :type axis_name: str

        :param positive_callback: The callback function to be called when the axis 
                                  transitions from 0 to 1.
        :type positive_callback: function

        :param negative_callback: The callback function to be called when the axis 
                                  transitions from 0 to -1.
        :type negative_callback: function        

        """
        # Detect rising edge (0 -> 1) for the axis
        if current_value == 1 and prev_value == 0:
            positive_callback()
            # self.get_logger().info(f'{axis_name} positive pressed')

        # Detect rising edge (0 -> -1) for the axis
        if current_value == -1 and prev_value == 0:
            negative_callback()
            # self.get_logger().info(f'{axis_name} negative pressed')

    def joy_callback(self, msg: Joy):
        """ Callback function for the joystick subscriber """

        # Get the button and axis values
        button_triangle = msg.buttons[BUTTON_TRIANGLE]
        button_circle = msg.buttons[BUTTON_CIRCLE]
        button_square = msg.buttons[BUTTON_SQUARE]
        button_cross = msg.buttons[BUTTON_CROSS]
        button_left_right = msg.axes[AXIS_LEFT_RIGHT]
        button_up_down = msg.axes[AXIS_UP_DOWN]

        self.handle_button(button_square,
                           self.prev_buttons[BUTTON_SQUARE],
                           'Square_button',
                           self.grip_open,
                           None
                           )

        self.handle_button(button_circle,
                           self.prev_buttons[BUTTON_CIRCLE],
                           'Circle_button',
                           self.grip_close,
                           None
                           )

        self.handle_axis(button_up_down,
                         self.prev_buttons[AXIS_UP_DOWN],
                         'Up_Down_axis',
                         self.lift_up,
                         self.lift_down
                         )

        self.handle_axis(button_left_right,
                         self.prev_buttons[AXIS_LEFT_RIGHT],
                         'Left_Right_axis',
                         self.arm_angle_increase,
                         self.arm_angle_decrease
                         )
        self.handle_button(button_triangle,
                           self.prev_buttons[BUTTON_TRIANGLE],
                           'Triangle_button',
                           self.claw_angle_increase,
                           None
                           )

        self.handle_button(button_cross,
                           self.prev_buttons[BUTTON_CROSS],
                           'Cross_button',
                           self.claw_angle_decrease,
                           None
                           )

        # Store the previous button states
        self.prev_buttons[BUTTON_TRIANGLE] = button_triangle
        self.prev_buttons[BUTTON_CIRCLE] = button_circle
        self.prev_buttons[BUTTON_CROSS] = button_cross
        self.prev_buttons[AXIS_LEFT_RIGHT] = button_left_right
        self.prev_buttons[BUTTON_SQUARE] = button_square
        self.prev_buttons[AXIS_UP_DOWN] = button_up_down

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


def main(args=None):
    """Main function to create the JoyControl node."""
    rclpy.init(args=args)
    joy_subscriber = JoyControl()
    try:
        rclpy.spin(joy_subscriber)
    except KeyboardInterrupt:
        joy_subscriber.get_logger().info("Joy subscriber node stopped by the user.")
    finally:
        joy_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
