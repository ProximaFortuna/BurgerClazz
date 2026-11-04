#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import math
import time

# x, y, theta (in radians) for each goal
GOALS = [
    (0.126, 0.708, 0.0),  # Goal1
    (1.93, 0.163, 0.0), # Goal2
    (1.24, 0.114, 0.0),  # Goal3
]

def make_goal(x, y, navigator):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.0
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0

    return goal

def main():
    rclpy.init()
    navigator = BasicNavigator()

    print("Waiting for Nav2 to fully activate...")
    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    print("Nav2 is active and ready to receive goals.")

    for i, (x, y, theta) in enumerate(GOALS):
        goal = make_goal(x, y, navigator)
        navigator.goToPose(goal)
        print(f"Sent goal {i + 1}: ({x}, {y}, {theta})")

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(f"Distance remaining: {feedback.distance_remaining:.2f} meters")
            time.sleep(0.1)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Goal reached successfully!")
        else:
            print("Failed to reach the goal.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()