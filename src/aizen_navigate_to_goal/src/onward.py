#!/usr/bin/env python3

# Mopel Kitele and Logan Purkiss

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class Onward(Node):

    def __init__(self):
        super().__init__('onward')

        # --------------------------------
        # Controller parameters
        # --------------------------------

        # Heading threshold:
        # Rotate in place if the desired direction
        # is more than 30 degrees from straight ahead.
        self.theta_threshold = math.radians(30.0)

        # Angular proportional gain
        self.kp_theta = 2

        # Forward cruising velocity
        self.cruise_velocity = 0.12  # m/s

        # Velocity saturation
        self.max_linear_velocity = 0.22   # m/s
        self.max_angular_velocity = 1.0   # rad/s

        # Minimum resultant field magnitude required to move
        self.min_field_magnitude = 0.01

        # Stop if navigation commands become stale
        self.command_timeout = 0.5  # seconds

        # --------------------------------
        # Latest navigation measurement
        # --------------------------------

        self.direction_angle = 0.0
        self.direction_mag = 0.0

        self.have_measurement = False
        self.last_direction_time = None

        # --------------------------------
        # ROS interfaces
        # --------------------------------

        # Expected message from find_vector:
        #
        # msg.data[0] = resultant potential-field magnitude
        # msg.data[1] = steering angle in radians
        #
        # Positive angle = turn left (counterclockwise)
        # Negative angle = turn right (clockwise)

        self.direction_sub = self.create_subscription(
            Float32MultiArray,
            '/direction',
            self.direction_callback,
            10
        )

        # Velocity commands to TurtleBot3
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 20 Hz control loop
        self.control_timer = self.create_timer(
            0.05,
            self.control_callback
        )

        self.get_logger().info('Onward controller started.')

    # --------------------------------
    # Direction callback
    # --------------------------------

    def direction_callback(self, msg):
        """
        Receives the resultant vector calculated by
        the find_vector node.

        Stores the latest magnitude and steering angle.
        """

        if len(msg.data) != 2:
            self.get_logger().warn(
                'Expected [magnitude, angle] from /direction.'
            )
            return

        magnitude = float(msg.data[0])
        angle = float(msg.data[1])

        # Reject invalid measurements
        if not math.isfinite(magnitude) or not math.isfinite(angle):
            return

        if magnitude < 0.0:
            return

        self.direction_mag = magnitude

        # Normalize steering angle to [-pi, pi]
        self.direction_angle = math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

        self.have_measurement = True
        self.last_direction_time = self.get_clock().now()

    # --------------------------------
    # Control callback
    # --------------------------------

    def control_callback(self):
        """
        Converts the desired potential-field direction
        into linear and angular velocity commands.

        Runs at 20 Hz.
        """

        # Default command is zero velocity
        cmd = Twist()

        # --------------------------------
        # Check navigation data
        # --------------------------------

        # Do not move until a valid measurement arrives
        if not self.have_measurement:
            self.cmd_vel_pub.publish(cmd)
            return

        # Stop if the latest measurement is too old
        elapsed = (
            self.get_clock().now() - self.last_direction_time
        ).nanoseconds / 1e9

        if elapsed < 0.0 or elapsed > self.command_timeout:
            self.cmd_vel_pub.publish(cmd)
            return

        # --------------------------------
        # Check resultant magnitude
        # --------------------------------

        # A zero resultant vector indicates either:
        # 1. The robot has reached a waypoint and must stop.
        # 2. The potential-field forces have nearly cancelled.
        #
        # In either case, command zero velocity.

        if self.direction_mag < self.min_field_magnitude:
            self.cmd_vel_pub.publish(cmd)
            return

        # --------------------------------
        # Compute heading error
        # --------------------------------

        # find_vector already provides the desired
        # steering angle in the robot's local frame.

        e_theta = self.direction_angle

        # --------------------------------
        # Angular proportional controller
        # --------------------------------

        omega_raw = self.kp_theta * e_theta

        # --------------------------------
        # Linear velocity controller
        # --------------------------------

        if abs(e_theta) >= self.theta_threshold:

            # Desired direction is too far from
            # the robot's current heading.
            #
            # Rotate in place before translating.

            v_raw = 0.0

        else:

            # Heading-dependent forward velocity scaling.
            #
            # At zero heading error:
            # g_theta = 1
            #
            # At the 30-degree threshold:
            # g_theta = 0

            g_theta = 1.0 - (
                abs(e_theta) / self.theta_threshold
            ) ** 2

            # Clamp scaling factor to [0, 1]
            g_theta = max(
                0.0,
                min(1.0, g_theta)
            )

            # Forward velocity decreases as
            # heading error increases.

            v_raw = self.cruise_velocity * g_theta

        # --------------------------------
        # Velocity saturation
        # --------------------------------

        # Only allow forward translation
        v_cmd = max(
            0.0,
            min(self.max_linear_velocity, v_raw)
        )

        # Limit angular velocity
        omega_cmd = max(
            -self.max_angular_velocity,
            min(self.max_angular_velocity, omega_raw)
        )

        # --------------------------------
        # Publish velocity command
        # --------------------------------

        cmd.linear.x = v_cmd
        cmd.angular.z = omega_cmd

        self.cmd_vel_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = Onward()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        # Stop robot before shutting down
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
