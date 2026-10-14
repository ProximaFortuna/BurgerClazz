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
import os
from ament_index_python.packages import get_package_share_directory

# Note: The following code is based on the potential field method for robot navigation. 
# The robot is attracted to the goal and repelled by obstacles detected by the LIDAR. 
# The final direction is a combination of these two vectors.
class FindVector(Node):

    def __init__(self):
        super().__init__('find_vector')

        self.robot_x = None
        self.robot_y = None

        self.obstacle_radius = 2  # radius where obstacles are considered (meters)
        self.min_valid_range = 0.16  # radius where too close lidar scan points are ignored (meters)
        self.repulsion_gain = 1.0  # Gain for the repulsion vector (Increase to make the robot more sensitive to obstacles)

        self.goal_tol = 0.01 # Tolerance for reaching the goal (meters)

        path = os.path.join(
            get_package_share_directory('aizen_navigate_to_goal'), 'goals.txt')
        self.goals = self.load_goals(path) # Load goals from a file
        self.current_goal_index = 0 # Current index to list of goals
        self.goal_gain = 50 # Gain for the attraction vector (Increase to make the robot more sensitive to goals)

        # Subscriber for fixed odom data
        self.odom_subscriber = self.create_subscription(
            Point,
            '/location',
            self.odom_callback,
            10
        )

        # Subscriber for raw Lidar data
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile_sensor_data
        )

        # Publisher for final direction, format [Magnitude, Angle]
        self.direction_publisher = self.create_publisher(
            Float32MultiArray,
            '/direction',
            10
        )

    # Retrieve fixed odom data
    def odom_callback(self, data):
        # Get the current position of the robot
        self.robot_x = data.x
        self.robot_y = data.y

    # Main function to process lidar data
    def lidar_callback(self, msg):

        # If no odom data, warn
        if self.robot_x is None or self.robot_y is None:
            self.get_logger().warn("No odometry data received yet.", throttle_duration_sec=2.0)
            return
        # If no goals, warn
        if not self.goals:
            self.get_logger().warn("No goals loaded.")
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
        self.range_max = min(self.obstacle_radius, self.range_max)

        # Initialize combined vector values
        total_vector_x = 0
        total_vector_y = 0

        # Loop through the Lidar scan range data and for ever reading that is within the range to be considered an obstacle calculate it's x and y
        # coords within the local frame. Scale by a factor that increases relevance the closer the object and add to the combined vector values.
        for i, r in enumerate(self.laser_ranges):
            if r <= self.range_max and r >= self.range_min:
                # Calculate the angle of the laser scan
                angle = self.angle_min + i * self.angle_increment

                # Calculate the x and y coordinates of the laser scan point
                x = r * np.cos(angle)
                y = r * np.sin(angle)

                # Calculate a scaling term that increases as the obstacle gets closer
                mag = self.repulsion_gain * (1 / r - 1 / self.obstacle_radius)

                # Calculate the vector from the laser scan point to the robot's position
                total_vector_x += -x / r * mag
                total_vector_y += -y / r * mag

        # Get the obstacle angle and magnitude for debugging purposes
        obstacle_angle = math.atan2(total_vector_y, total_vector_x) % (2 * math.pi)
        obstacle_mag = math.hypot(total_vector_x, total_vector_y)

        # Get the x and y parts of the vector pointing to the current goal
        goal_x, goal_y = self.goals[self.current_goal_index]
        goal_vector_x = goal_x - self.robot_x
        goal_vector_y = goal_y - self.robot_y

        # Get the magnitude of the goal distance for tolerance comparison
        goal_mag = math.hypot(goal_vector_x, goal_vector_y)

        # If we are close enough to the goal then move to the next one
        if goal_mag < self.goal_tol:
            self.get_logger().info(f"Reached goal {self.current_goal_index + 1}: ({goal_x}, {goal_y})")
            self.current_goal_index += 1

            # If we are at the last goal, don't do anything anymore
            if self.current_goal_index >= len(self.goals):
                self.get_logger().info("All goals reached.")
                return

            # If not the last goal, wait 10 seconds and then continue to the next goal
            else:
                wait_time = 10  # seconds
                self.get_logger().info(f"Waiting for {wait_time} seconds before moving to the next goal...")
                rclpy.sleep(wait_time)
                goal_x, goal_y = self.goals[self.current_goal_index]
                self.get_logger().info(f"Moving to next goal {self.current_goal_index + 1}: ({goal_x}, {goal_y})")
                goal_vector_x = goal_x - self.robot_x
                goal_vector_y = goal_y - self.robot_y

        # Scale the goal vector to make it more important
        scaled_goal_vector_x = goal_vector_x * self.goal_gain
        scaled_goal_vector_y = goal_vector_y * self.goal_gain

        # Get the final combined vector x and y
        total_vector_x += scaled_goal_vector_x
        total_vector_y += scaled_goal_vector_y

        # Calculate the final direction and magnitude
        final_angle = math.atan2(total_vector_y, total_vector_x) % (2 * math.pi)
        final_mag = math.hypot(total_vector_x, total_vector_y)

        # Create a Float32MultiArray message for the direction
        direction_msg = Float32MultiArray()
        direction_msg.data = [float(final_mag), float(final_angle)]
        self.direction_publisher.publish(direction_msg)

    # Function to read goals from a text file and store them as a list
    def load_goals(self, filename):
        goals = []
        try:
            with open(filename) as f:
                for line_number, line in enumerate(f, start=1):
                    line = line.split('#')[0].strip()  # Remove comments and whitespace
                    if not line:  # Skip empty lines
                        continue
                    parts = line.replace(',', ' ').split()  # Replace commas with spaces and split
                    if len(parts) != 2:
                        self.get_logger().error(f"Invalid goal format on line {line_number}: '{line}'")
                        continue
                    try:
                        goals.append((float(parts[0]), float(parts[1])))
                    except ValueError:
                        self.get_logger().error(f"Non-numeric goal values on line {line_number}: '{line}'")
        except FileNotFoundError:
            self.get_logger().error(f"Goals file '{filename}' not found.")
        self.get_logger().info(f"Loaded goals: {goals}")
        return goals

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