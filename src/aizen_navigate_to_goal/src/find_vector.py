#!/usr/bin/env python3

# Logan Purkiss and Mopel Kitele
import math

import numpy as np
import cv2
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class FindVector(Node):

    def __init__(self):
        super().__init__('find_vector')

        self.robot_x = None
        self.robot_y = None

        self.inflation_radius = 2  # meters
        self.min_valid_range = 0.16  # meters
        self.repulsion_gain = 1.0  # Gain for the repulsion vector

        # Create a subscriber for the image topic
        self.odom_subscriber = self.create_subscription(
            Point,
            '/location',
            self.odom_callback,
            10
        )

        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile_sensor_data
        )

        self.direction_publisher = self.create_publisher(
            Float32MultiArray,
            '/direction',
            10
        )

    def odom_callback(self, data):
        # Get the current position of the robot
        self.robot_x = data.x
        self.robot_y = data.y

    def lidar_callback(self, msg):

        if not hasattr(self, 'robot_x') or not hasattr(self, 'robot_y'):
            self.get_logger().warn("No odometry data received yet.")
            return

         # Store the laser scan values from the message
        self.laser_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max

        # Define the valid range for the laser scan
        self.range_min = max(self.min_valid_range, self.range_min)
        self.range_max = min(self.inflation_radius, self.range_max)

        total_vector_x = 0
        total_vector_y = 0

        for i, r in enumerate(self.laser_ranges):
            if r <= self.range_max and r >= self.range_min:
                # Calculate the angle of the laser scan
                angle = self.angle_min + i * self.angle_increment

                # Calculate the x and y coordinates of the laser scan point
                x = r * np.cos(angle)
                y = r * np.sin(angle)

                mag = self.repulsion_gain * (1 / r - 1 / self.inflation_radius) / (r ** 2)

                # Calculate the vector from the laser scan point to the robot's position
                total_vector_x += -x / r * mag
                total_vector_y += -y / r * mag

        obstacle_angle = math.atan2(total_vector_y, total_vector_x)
        obstacle_mag = math.hypot(total_vector_x, total_vector_y)

        # Create a Float32MultiArray message for the direction
        direction_msg = Float32MultiArray()
        direction_msg.data = [float(obstacle_mag), float(obstacle_angle)]
        self.direction_publisher.publish(direction_msg)

def main(args=None):
    rclpy.init(args=args)
    find_vector_node = FindVector()
    try:
        rclpy.spin(find_vector_node)
    except KeyboardInterrupt:
        pass
    finally:
        find_vector_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()