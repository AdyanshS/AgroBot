#!/usr/bin/env python3

"""
This module contains a ROS node that publishes IMU data from a BNO08x sensor.

Overview:
- The `BNO08xNode` class is a ROS node that initializes the BNO08x sensor and publishes IMU data.
- It reads acceleration, gyroscope, and quaternion data from the sensor and publishes it as an `Imu` message.

Publishers:
- `imu/data` (Imu): Publishes IMU data including linear acceleration, angular velocity, and orientation.

Transforms:
- `base_link` to `imu_link`: Publishes the transform from the base_link to the imu_link.
"""


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from diagnostic_msgs.msg import DiagnosticStatus
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C


class BNO08xNode(Node):
    def __init__(self):
        super().__init__('bno08x')

        # Publisher for IMU data
        self.raw_pub = self.create_publisher(Imu, 'imu/data', 10)
        # Uncomment if needed:
        # self.mag_pub = self.create_publisher(MagneticField, 'bno08x/mag', 10)
        # self.status_pub = self.create_publisher(DiagnosticStatus, 'bno08x/status', 10)

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('bno08x node launched.')

        # Initialize I2C communication
        # for the r-pi this will be  board.SCL and board.SDA
        i2c = busio.I2C(board.SCL_1, board.SDA_1)
        # address fpr the BNO080 (0x4b) BNO085 or BNO08X (0x4a)
        self.bno = BNO08X_I2C(i2c, address=0x4A)

        # Enable required features
        self.bno.initialize()
        print("IMU Initialized")

        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        # Timer to run at 170Hz (0.005882353)
        self.timer = self.create_timer(0.00588, self.run)

    def run(self):
        raw_msg = Imu()
        raw_msg.header.stamp = self.get_clock().now().to_msg()
        raw_msg.header.frame_id = 'base_link'

        # Read acceleration data
        accel_x, accel_y, accel_z = self.bno.acceleration
        raw_msg.linear_acceleration.x = accel_x
        raw_msg.linear_acceleration.y = accel_y
        raw_msg.linear_acceleration.z = accel_z

        # Read gyroscope data
        gyro_x, gyro_y, gyro_z = self.bno.gyro
        raw_msg.angular_velocity.x = gyro_x
        raw_msg.angular_velocity.y = gyro_y
        raw_msg.angular_velocity.z = gyro_z

        # Read quaternion data
        quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
        raw_msg.orientation.w = quat_real
        raw_msg.orientation.x = quat_i
        raw_msg.orientation.y = quat_j
        raw_msg.orientation.z = quat_k

        # Publish the IMU data
        self.raw_pub.publish(raw_msg)

        # Uncomment to publish the transform from base_link to imu_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat_i
        t.transform.rotation.y = quat_j
        t.transform.rotation.z = quat_k
        t.transform.rotation.w = quat_real
        self.tf_broadcaster.sendTransform(t)

        # Uncomment if you want to publish the magnetic field and status messages
        # mag_msg = MagneticField()
        # mag_x, mag_y, mag_z = self.bno.magnetic
        # mag_msg.header.stamp = self.get_clock().now().to_msg()
        # mag_msg.magnetic_field.x = mag_x
        # mag_msg.magnetic_field.y = mag_y
        # mag_msg.magnetic_field.z = mag_z
        # mag_msg.magnetic_field_covariance[0] = -1
        # self.mag_pub.publish(mag_msg)

        # status_msg = DiagnosticStatus()
        # status_msg.level = bytes([0])
        # status_msg.name = "bno08x IMU"
        # status_msg.message = ""
        # self.status_pub.publish(status_msg)

    def shutdown(self):
        self.get_logger().info('bno08x node finished')


def main(args=None):
    rclpy.init(args=args)
    node = BNO08xNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
