#!/usr/bin/env python3

# Logan Purkiss and Mopel Kitele
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


class GetObjectDistance(Node):

    def __init__(self):
        super().__init__('get_object_distance')

        self.target_img_x = None
        self.target_img_y = None
        self.target_angle = None
        self.target_distance = None
        self.jump_count = 0

        # Create a subscriber for the centroid topic
        self.centroid_subscriber = self.create_subscription(
            Point,
            '/tracking/centroid',
            self.centroid_callback,
            10
        )

        # Create a subscriber for the laser scan topic
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile_sensor_data
        )
        
        # Create a publisher for the distance values
        self.distance_publisher = self.create_publisher(
            Float32MultiArray,
            '/tracking/distance',
            10
        )
        self.get_logger().info("GetObjectDistance node has been started.")

    def centroid_callback(self, msg):
        # Store the target centroid values from the message
        self.target_img_x = msg.x
        self.target_img_y = msg.y

    def laser_callback(self, msg):

        if self.target_img_x is None:
            self.get_logger().warn("No target centroid received yet.")
            return

        # Store the laser scan values from the message
        self.laser_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max

        # Define the valid range for the laser scan
        self.range_min = max(0.16, self.range_min)
        self.range_max = min(80, self.range_max)

        # Calculate the angle of the target in radians
        fov = 62.2 * (np.pi / 180)  # Convert FOV to radians
        img_width = 320  # Image width in pixels
        angle = (self.target_img_x - (img_width / 2)) * (fov / img_width)  # Angle in radians
        self.target_angle = angle
        angle = (angle - self.angle_min) % (2 * np.pi) + self.angle_min  # Normalize angle to [0, 2π] for indexing

        # Calculate the index of the laser scan range corresponding to the target angle
        n = len(self.laser_ranges)
        if n == 0:
            self.get_logger().warn("Laser scan ranges are empty.")
            return

        index = int(round((angle - self.angle_min) / self.angle_increment)) % n

        # Get the distance to the target from the laser scan ranges
        half = max(2, int(np.deg2rad(2) / self.angle_increment))  # Half window size for averaging
        window = [self.laser_ranges[(index + k) % n] for k in range(-half, half + 1)]  # Get a window of ranges around the target index 
        valid = [r for r in window if self.range_min < r < self.range_max]
        if not valid:
            self.get_logger().warn("No valid laser scan ranges found in the window around the target angle.")
            return  # No valid ranges in the window

        self.target_distance = float(np.percentile(valid, 25))  # Use the 25th percentile to reduce the effect of outliers

        # Log the target distance
        self.get_logger().info(f"Target distance: {self.target_distance:.2f} meters")

        # Publish the target distance to the distance topic
        distance_msg = Float32MultiArray()
        distance_msg.data = [float(self.target_angle), float(self.target_distance)]
        self.distance_publisher.publish(distance_msg)

def main(args=None):
    rclpy.init(args=args)
    object_distance_node = GetObjectDistance()
    try:
        rclpy.spin(object_distance_node)
    except KeyboardInterrupt:
        pass
    finally:
        object_distance_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
