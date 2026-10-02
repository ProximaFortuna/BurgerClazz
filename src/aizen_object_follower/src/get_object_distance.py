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


class GetObjectDistance(Node):

    def __init__(self):
        super().__init__('get_object_distance')

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
            10
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
        # Store the laser scan values from the message
        self.laser_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max

        # Define the valid range for the laser scan
        self.spec_range.min = 0.16
        self.spec_range.max = 80

        # Ensure the range values are within the specified limits
        if self.range_max > self.spec_range.max:
            self.range_max = self.spec_range.max
        if self.range_min < self.spec_range.min:
            self.range_min = self.spec_range.min

        # Filter out invalid laser scan ranges
        filtered_ranges = [r for r in self.laser_ranges if r < self.range_max and r > self.range_min]

        # Calculate the angle of the target in radians
        angle = (self.target_img_x - 160) * (np.pi / 320)
        self.target_angle = angle

        # Calculate the index of the laser scan range corresponding to the target angle
        index = int((angle - self.angle_min) / self.angle_increment)

        # Get the distance to the target from the laser scan ranges
        if 0 <= index < len(filtered_ranges):
            self.target_distance = np.mean(filtered_ranges[index-2:index+3])  # Average over a small range to reduce noise
        # Log the target distance
        self.get_logger().info(f"Target distance: {self.target_distance:.2f} meters")

        # Publish the target distance to the distance topic
        distance_msg = Float32MultiArray()
        distance_msg.data = [self.target_angle, self.target_distance]
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
