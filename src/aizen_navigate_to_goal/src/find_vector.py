#!/usr/bin/env python3

# Logan Purkiss and Mopel Kitele
import math

import numpy as np
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
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
        self.robot_yaw = None

        self.follow_side = 0.0
        self.follow_wall = False
        self.follow_start_d = 0.0
        self.follow_start_mag = 10
        self.min_push = 1.0
        self.needed_progress = 0.2
        self.follow_distance = 0.1
        self.keepaway_band = 0.15

        self.wait_until = None

        self.obstacle_radius = 2  # radius where obstacles are considered (meters)
        self.min_valid_range = 0.16  # radius where too close lidar scan points are ignored (meters)
        self.repulsion_gain = 1.0  # Gain for the repulsion vector (Increase to make the robot more sensitive to obstacles)

        self.goal_tol = 0.05 # Tolerance for reaching the goal (meters)

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
        self.robot_yaw = data.z

    def reset_follow(self):
        self.follow_wall = False

    def stop(self):
        direction_msg = Float32MultiArray()
        direction_msg.data = [0.0, 0.0]
        self.direction_publisher.publish(direction_msg)

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

        # Initialize message
        direction_msg = Float32MultiArray()

        # Stop when at last goal
        if self.current_goal_index >= len(self.goals):
            self.stop()
            return

        # Wait between goals
        if self.wait_until is not None:
            if self.get_clock().now() < self.wait_until:
                self.stop()
                return
            self.wait_until = None
            self.reset_follow()

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
        push_vector_x = 0
        push_vector_y = 0

        nearest_r = float('inf')
        nearest_angle = 0.0

        # Loop through the Lidar scan range data and for ever reading that is within the range to be considered an obstacle calculate it's x and y
        # coords within the local frame. Scale by a factor that increases relevance the closer the object and add to the combined vector values.
        for i, r in enumerate(self.laser_ranges):
            if r <= self.range_max and r >= self.range_min:
                # Calculate the angle of the laser scan
                angle = self.angle_min + i * self.angle_increment

                # Calculate the x and y coordinates of the laser scan point
                x = r * np.cos(angle)
                y = r * np.sin(angle)

                if r < nearest_r:
                    nearest_r = r
                    nearest_angle = angle

                # Calculate a scaling term that increases as the obstacle gets closer
                mag = self.repulsion_gain * (1 / r - 1 / self.obstacle_radius)

                # Calculate the vector from the laser scan point to the robot's position
                push_vector_x += -x / r * mag
                push_vector_y += -y / r * mag

        # Get the obstacle angle and magnitude for debugging purposes
        obstacle_angle = math.atan2(push_vector_y, push_vector_x)
        obstacle_mag = math.hypot(push_vector_x, push_vector_y)

        # Get the x and y parts of the vector pointing to the current goal
        goal_x, goal_y = self.goals[self.current_goal_index]
        dx = goal_x - self.robot_x
        dy = goal_y - self.robot_y
        c, s = math.cos(self.robot_yaw), math.sin(self.robot_yaw)
        goal_vector_x = c * dx + s * dy
        goal_vector_y = -s * dx + c * dy

        # Get the magnitude of the goal distance for tolerance comparison
        goal_mag = math.hypot(goal_vector_x, goal_vector_y)

        # If we are close enough to the goal then move to the next one
        if goal_mag < self.goal_tol:
            self.get_logger().info(f"Reached goal {self.current_goal_index + 1}: ({goal_x}, {goal_y})")
            self.current_goal_index += 1

            # If we are at the last goal, don't do anything anymore
            if self.current_goal_index >= len(self.goals):
                self.get_logger().info("All goals reached.")
                self.stop()
                return

            # If not the last goal, wait 10 seconds and then continue to the next goal
            else:
                self.wait_until = self.get_clock().now() + Duration(seconds=10)

        # Scale the unit goal vector to make it magnitude of 50
        goal_ux = goal_vector_x / goal_mag
        goal_uy = goal_vector_y / goal_mag

        scaled_goal_vector_x = goal_ux * self.goal_gain
        scaled_goal_vector_y = goal_uy * self.goal_gain

        # Get the final combined vector x and y
        total_vector_x = scaled_goal_vector_x + push_vector_x
        total_vector_y = scaled_goal_vector_y + push_vector_y

        # Calculate the final direction and magnitude
        final_angle = math.atan2(total_vector_y, total_vector_x)
        final_mag = math.hypot(total_vector_x, total_vector_y)

        # Progress checking

        # Get time
        now = self.get_clock().now().nanoseconds * 1.e-9

        # If we should follow wall but we aren't then start following
        if not self.follow_wall and final_mag < self.follow_start_mag and obstacle_mag > self.min_push:
            cross = goal_uy * push_vector_x - push_vector_y * goal_ux
            self.follow_side = 1.0 if cross >= 0 else -1.0
            self.follow_wall = True
            self.follow_start_d = goal_mag

        # If we have made enough progress then stop following the wall
        if self.follow_wall and goal_mag < self.follow_start_d - self.needed_progress:
            self.follow_wall = False

        # If we are supposed to follow the wall, then follow a vector tangent to the wall
        if self.follow_wall:
            if nearest_r < float('inf'):
                nearest_ux = math.cos(nearest_angle)
                nearest_uy = math.sin(nearest_angle)

                tx = self.follow_side * nearest_uy
                ty = self.follow_side * -nearest_ux

                err = (nearest_r - self.follow_distance) / self.keepaway_band
                err = max(-1.0, min(1.0, err))
                radial = self.goal_gain * err

                total_vector_x = tx * self.goal_gain + nearest_ux * radial
                total_vector_y = ty * self.goal_gain + nearest_uy * radial
                final_angle = math.atan2(total_vector_y, total_vector_x)
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