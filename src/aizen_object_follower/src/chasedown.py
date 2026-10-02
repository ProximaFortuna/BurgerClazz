#!/usr/bin/env python3

# Mopel Kitele and Logan Purkiss

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class ChaseObject(Node):

    def __init__(self):
        super().__init__('chase_object')

        # --------------------------------
        # Controller parameters
        # --------------------------------

        # Desired distance from the object: 1 foot
        self.desired_distance = 0.3048  # meters

        # If heading error is greater than 30 degrees,
        # stop translating and focus entirely on rotation.
        self.theta_threshold = math.radians(30.0)

        # Proportional gains
        # These are starting values and WILL need tuning.
        self.kp_theta = 1.0
        self.kp_dist = 0.5

        # Velocity saturation
        self.max_linear_velocity = 0.22   # m/s
        self.max_angular_velocity = 1.0   # rad/s

        # --------------------------------
        # Latest object measurement
        # --------------------------------

        self.object_angle = 0.0
        self.object_distance = 0.0

        self.have_measurement = False

        # --------------------------------
        # ROS interfaces
        # --------------------------------

        # Expected message:
        # msg.data[0] = object angle in radians
        # msg.data[1] = object distance in meters
        self.object_sub = self.create_subscription(
            Float32MultiArray,
            '/tracking/distance',
            self.object_callback,
            10
        )

        # Velocity commands to TurtleBot
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

        self.get_logger().info('Chase object controller started.')

    def object_callback(self, msg):
        """
        Stores the most recent object measurement.
        This callback does NOT directly control the robot.
        """

        if len(msg.data) < 2:
            self.get_logger().warn(
                'Received malformed /tracking/distance message.'
            )
            return

        self.object_angle = msg.data[0]
        self.object_distance = msg.data[1]

        self.have_measurement = True

    def control_callback(self):
        """
        Runs the actual chasedown controller at 20 Hz.
        """

        cmd = Twist()

        # Do not move until at least one valid measurement
        # has been received.
        if not self.have_measurement:
            self.cmd_vel_pub.publish(cmd)
            return

        # --------------------------------
        # Compute errors
        # --------------------------------

        # Desired heading is straight ahead: 0 radians.
        #
        # Partner's convention:
        # positive object angle = object is to the right
        #
        # Therefore:
        # object right  -> positive theta_obj
        # e_theta       -> negative
        # omega         -> negative
        # robot rotates clockwise/right
        e_theta = -self.object_angle

        # Positive distance error means the object is too far away.
        # Negative distance error means the object is too close.
        e_dist = self.object_distance - self.desired_distance

        # --------------------------------
        # Angular proportional controller
        # --------------------------------

        omega_raw = self.kp_theta * e_theta

        # --------------------------------
        # Linear proportional controller
        # --------------------------------

        if abs(e_theta) > self.theta_threshold:
            # Object is too far off-center.
            # Rotate first before translating.
            v_raw = 0.0

        else:
            # Heading-dependent forward velocity scaling.
            #
            # Perfect heading:
            # e_theta = 0  -> g_theta = 1
            #
            # At 30-degree threshold:
            # |e_theta| = theta_threshold -> g_theta = 0
            #
            # The squared relationship lets the robot retain
            # more forward speed for smaller heading errors,
            # then slows it aggressively near the threshold.
            g_theta = 1.0 - (
                abs(e_theta) / self.theta_threshold
            ) ** 2

            # Defensive clamp to guarantee:
            # 0 <= g_theta <= 1
            g_theta = max(
                0.0,
                min(1.0, g_theta)
            )

            v_raw = (
                self.kp_dist
                * e_dist
                * g_theta
            )

        # --------------------------------
        # Velocity saturation
        # --------------------------------

        v_cmd = max(
            -self.max_linear_velocity,
            min(self.max_linear_velocity, v_raw)
        )

        omega_cmd = max(
            -self.max_angular_velocity,
            min(self.max_angular_velocity, omega_raw)
        )

        # --------------------------------
        # Build and publish Twist command
        # --------------------------------

        cmd.linear.x = v_cmd
        cmd.angular.z = omega_cmd

        self.cmd_vel_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = ChaseObject()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # Send one final zero-velocity command before shutting down.
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
