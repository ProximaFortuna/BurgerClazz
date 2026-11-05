#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import math
import time

# x, y, theta (in radians) for each goal
GOALS = [
    (2.22, 0.854, 0.0), # Goal1
    (1.92, 0.137, 0.0),  # Goal2
    (0.0506, 1.3, 0.0) # Goal3
]

def make_goal(x, y, theta, navigator):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    goal.pose.orientation.z = math.sin(theta / 2.0)
    goal.pose.orientation.w = math.cos(theta / 2.0)

    return goal

MAX_RETRIES = 3  # Maximum number of retries for each goal

def main():
    rclpy.init()
    navigator = BasicNavigator()

    print("Waiting for Nav2 to fully activate...")
    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    print("Nav2 is active and ready to receive goals.")

    for i, (x, y, theta) in enumerate(GOALS):
        for attempt in range(MAX_RETRIES):
            print(f"Attempt {attempt + 1} for goal {i + 1}: ({x}, {y}, {theta})")
            goal = make_goal(x, y, theta, navigator)
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
                break

            else:
                print("Failed to reach the goal.")

                if attempt < MAX_RETRIES - 1:
                    print("Retrying...")
                    time.sleep(1)

                else:
                    print("Max retries reached. Moving to the next goal.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()