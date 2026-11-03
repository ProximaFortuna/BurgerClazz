from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
import math
import time

# x, y, theta (in radians) for each goal
GOALS = [
    (1.0, 0.0, 0.0),  # Goal1
    (-1.0, 0.0, 0.0), # Goal2
    (0.0, 1.0, 0.0),  # Goal3
]

def make_goal(x, y, theta, navigator):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = float(x)
    goal.pose.position.y = float(y)
    yaw = math.radians(theta)
    goal.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.orientation.w = math.cos(yaw / 2.0)
    return goal

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    goals = [make_goal(x, y, theta, navigator) for x, y, theta in GOALS]

    navigator.followWaypoints(goals)

    # Wait for the robot to reach the goal
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"Distance remaining: {feedback.distance_remaining:.2f} meters")
        time.sleep(0.5)

    result = navigator.getResult()
    if result == BasicNavigator.Result.SUCCEEDED:
        print("Goal reached successfully!")
    else:
        print("Failed to reach the goal.")

    rclpy.shutdown()

    if __name__ == '__main__':
        main()