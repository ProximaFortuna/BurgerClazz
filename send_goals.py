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

def make_goal(x, y, theta, navigator):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    goal.pose.orientation.z = math.sin(theta / 2.0)
    goal.pose.orientation.w = math.cos(theta / 2.0)
    return goal

def main():
    rclpy.init()
    navigator = BasicNavigator()

    print("Waiting for Nav2 to fully activate...")
    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    print("Nav2 is active and ready to receive goals.")

    goals = [make_goal(x, y, theta, navigator) for x, y, theta in GOALS]

    navigator.followWaypoints(goals)

    # Wait for the robot to reach the goal
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"Executing current goal: {feedback.current_waypoint + 1}/{len(goals)}")
        time.sleep(0.5)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Goal reached successfully!")
    else:
        print("Failed to reach the goal.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()