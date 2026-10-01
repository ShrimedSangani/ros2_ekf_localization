#!/usr/bin/env python3
"""Add configurable synthetic Gaussian noise to Gazebo odometry and IMU data.

Run this node alongside Gazebo. It subscribes to ``/odom`` and ``/imu`` and
publishes noisy copies on ``/odom_noisy`` and ``/imu_noisy``. The original
messages are copied before modification, so timestamps and frame names are
preserved.
"""

import math
from copy import deepcopy

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def quaternion_from_yaw(yaw):
    """Return a quaternion representing a rotation about the z axis."""
    half_yaw = 0.5 * yaw
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def multiply_quaternions(first, second):
    """Return the Hamilton product of two quaternions in x-y-z-w order."""
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


class SyntheticNoiseWrapper(Node):
    """Republish odometry and IMU messages with independent Gaussian noise."""

    def __init__(self):
        super().__init__("synthetic_noise_wrapper")

        self.declare_parameter("odom_input_topic", "/odom")
        self.declare_parameter("odom_output_topic", "/odom_noisy")
        self.declare_parameter("imu_input_topic", "/imu")
        self.declare_parameter("imu_output_topic", "/imu_noisy")
        self.declare_parameter("random_seed", -1)

        # Odometry noise is expressed in metres, radians, metres/second, and
        # radians/second. The default values are deliberately modest.
        self.declare_parameter("odom_position_xy_stddev", 0.1)
        self.declare_parameter("odom_yaw_stddev", 0.05)
        self.declare_parameter("odom_linear_velocity_stddev", 0.05)
        self.declare_parameter("odom_angular_velocity_stddev", 0.03)

        # IMU noise is expressed in radians/second and metres/second^2.
        self.declare_parameter("imu_orientation_yaw_stddev", 0.01)
        self.declare_parameter("imu_angular_velocity_stddev", 0.01)
        self.declare_parameter("imu_linear_acceleration_stddev", 0.05)

        seed = self.get_parameter("random_seed").value
        self.random = np.random.default_rng(None if seed < 0 else seed)

        self.odom_position_xy_stddev = self._nonnegative_parameter(
            "odom_position_xy_stddev"
        )
        self.odom_yaw_stddev = self._nonnegative_parameter("odom_yaw_stddev")
        self.odom_linear_velocity_stddev = self._nonnegative_parameter(
            "odom_linear_velocity_stddev"
        )
        self.odom_angular_velocity_stddev = self._nonnegative_parameter(
            "odom_angular_velocity_stddev"
        )
        self.imu_orientation_yaw_stddev = self._nonnegative_parameter(
            "imu_orientation_yaw_stddev"
        )
        self.imu_angular_velocity_stddev = self._nonnegative_parameter(
            "imu_angular_velocity_stddev"
        )
        self.imu_linear_acceleration_stddev = self._nonnegative_parameter(
            "imu_linear_acceleration_stddev"
        )

        odom_input = self.get_parameter("odom_input_topic").value
        odom_output = self.get_parameter("odom_output_topic").value
        imu_input = self.get_parameter("imu_input_topic").value
        imu_output = self.get_parameter("imu_output_topic").value

        self.odom_publisher = self.create_publisher(Odometry, odom_output, 10)
        self.imu_publisher = self.create_publisher(Imu, imu_output, 10)
        self.create_subscription(
            Odometry,
            odom_input,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            imu_input,
            self.imu_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Noise wrapper: {odom_input} -> {odom_output}, "
            f"{imu_input} -> {imu_output}"
        )

    def _nonnegative_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if value < 0.0:
            raise ValueError(f"Parameter '{name}' must be non-negative")
        return value

    def _normal(self, standard_deviation, size=None):
        return self.random.normal(0.0, standard_deviation, size=size)

    def odom_callback(self, message):
        noisy = deepcopy(message)
        noisy.pose.pose.position.x += self._normal(self.odom_position_xy_stddev)
        noisy.pose.pose.position.y += self._normal(self.odom_position_xy_stddev)

        yaw_noise = self._normal(self.odom_yaw_stddev)
        original_orientation = noisy.pose.pose.orientation
        original_quaternion = (
            original_orientation.x,
            original_orientation.y,
            original_orientation.z,
            original_orientation.w,
        )
        noise_quaternion = quaternion_from_yaw(yaw_noise)
        noisy_quaternion = multiply_quaternions(original_quaternion, noise_quaternion)
        (
            noisy.pose.pose.orientation.x,
            noisy.pose.pose.orientation.y,
            noisy.pose.pose.orientation.z,
            noisy.pose.pose.orientation.w,
        ) = noisy_quaternion

        noisy.twist.twist.linear.x += self._normal(
            self.odom_linear_velocity_stddev
        )
        noisy.twist.twist.angular.z += self._normal(
            self.odom_angular_velocity_stddev
        )

        noisy.pose.covariance[0] += self.odom_position_xy_stddev**2
        noisy.pose.covariance[7] += self.odom_position_xy_stddev**2
        noisy.pose.covariance[35] += self.odom_yaw_stddev**2
        noisy.twist.covariance[0] += self.odom_linear_velocity_stddev**2
        noisy.twist.covariance[35] += self.odom_angular_velocity_stddev**2
        self.odom_publisher.publish(noisy)

    def imu_callback(self, message):
        noisy = deepcopy(message)

        angular_noise = self._normal(self.imu_angular_velocity_stddev, size=3)
        noisy.angular_velocity.x += angular_noise[0]
        noisy.angular_velocity.y += angular_noise[1]
        noisy.angular_velocity.z += angular_noise[2]

        acceleration_noise = self._normal(
            self.imu_linear_acceleration_stddev, size=3
        )
        noisy.linear_acceleration.x += acceleration_noise[0]
        noisy.linear_acceleration.y += acceleration_noise[1]
        noisy.linear_acceleration.z += acceleration_noise[2]

        # A covariance whose first element is -1 means that orientation is
        # unavailable. In that case, leave the orientation untouched.
        if noisy.orientation_covariance[0] >= 0.0:
            yaw_noise = self._normal(self.imu_orientation_yaw_stddev)
            original_orientation = noisy.orientation
            original_quaternion = (
                original_orientation.x,
                original_orientation.y,
                original_orientation.z,
                original_orientation.w,
            )
            noisy_quaternion = multiply_quaternions(
                original_quaternion, quaternion_from_yaw(yaw_noise)
            )
            (
                noisy.orientation.x,
                noisy.orientation.y,
                noisy.orientation.z,
                noisy.orientation.w,
            ) = noisy_quaternion
            noisy.orientation_covariance[8] += self.imu_orientation_yaw_stddev**2

        for index in (0, 4, 8):
            noisy.angular_velocity_covariance[index] += (
                self.imu_angular_velocity_stddev**2
            )
            noisy.linear_acceleration_covariance[index] += (
                self.imu_linear_acceleration_stddev**2
            )
        self.imu_publisher.publish(noisy)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticNoiseWrapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()