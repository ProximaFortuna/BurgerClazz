#!/usr/bin/env python3

# Mopel Kitele and Logan Purkiss

import math

import numpy as np
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

        # Proportional gains
        # These are starting values and WILL need tuning.
        self.kp_theta = 2
        self.kp_dist = 0.5

        self.max_linear_velocity = 0.22   # m/s
        self.max_angular_velocity = 1.0   # rad/s

        self.angle_deadband = 0.1  # radians
        self.distance_deadband = 0.05  # meters
        self.drive_angle_limit = 0.5  # radians

        self.timeout = 1

        # --------------------------------
        # Latest object measurement
        # --------------------------------

        self.object_angle = None
        self.object_distance = None

        self.last_msg_time = None

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

        self.last_msg_time = self.get_clock().now()

    def stop(self):
        """
        Stops the robot by publishing a zero-velocity command.
        """
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def control_callback(self):
        """
        Runs the actual chasedown controller at 20 Hz.
        """

        now = self.get_clock().now()

        # Do not move until at least one valid measurement
        # has been received.
        if self.last_msg_time is None or (now - self.last_msg_time).nanoseconds / 1e9 > self.timeout:
            self.stop()
            return

        # --------------------------------
        # Compute errors
        # --------------------------------

        e_theta = -self.object_angle
        e_theta = (e_theta + np.pi) % (2 * np.pi) - np.pi  # Wrap to [-pi, pi]
        e_dist = self.object_distance - self.desired_distance

        cmd = Twist()

        if abs(e_theta) > self.angle_deadband:
            cmd.angular.z = float(np.clip(self.kp_theta * e_theta, -self.max_angular_velocity, self.max_angular_velocity))

        if abs(e_dist) > self.distance_deadband and abs(e_theta) <= self.drive_angle_limit:
            lin = float(np.clip(self.kp_dist * e_dist, -self.max_linear_velocity, self.max_linear_velocity))
            cmd.linear.x = lin * max(0.0, np.cos(e_theta))  # Reduce forward speed when turning

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
