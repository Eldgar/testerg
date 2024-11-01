import time
from copy import deepcopy
from math import pi

from geometry_msgs.msg import PoseStamped, Twist
from rclpy.duration import Duration
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Define destinations
destinations = {
    "pre_loading_position": [2.25, 0.17, 1.5],
    "loading_position": [2.29, -1.3, 1.0],
    "shipping_position": [-0.6, 1.5, 1.0]
}

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set the initial pose of the robot
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -3.2
    initial_pose.pose.position.y = 0.17
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    # Define the sequence of destination names
    destination_sequence = ["pre_loading_position", "loading_position", "shipping_position"]

    for destination in destination_sequence:
        # Set up the pose for the current destination
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.header.stamp = navigator.get_clock().now().to_msg()
        target_pose.pose.position.x = destinations[destination][0]
        target_pose.pose.position.y = destinations[destination][1]
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = destinations[destination][2]

        print(f'Navigating to {destination}...')
        navigator.goToPose(target_pose)

        # Wait until task is complete at each destination
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                eta_seconds = feedback.estimated_time_remaining.nanoseconds / 1e9
                print(f'Estimated time of arrival at {destination}: {eta_seconds:.0f} seconds.')

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'Successfully reached {destination}.')
            # If this is the specific coordinate, perform the custom action
            if destination == "loading_position":
                print(f'Performing custom action at {destination}...')
                perform_custom_action(navigator)
        elif result == TaskResult.CANCELED:
            print(f'Navigation to {destination} was canceled.')
            navigator.goToPose(initial_pose)
            break
        elif result == TaskResult.FAILED:
            print(f'Failed to reach {destination}. Exiting.')
            exit(-1)

    print('Completed all destinations. Exiting.')
    exit(0)

def perform_custom_action(navigator):
    # Rotate the robot 180 degrees by publishing cmd_vel messages
    cmd_vel_pub = navigator.create_publisher(Twist, 'cmd_vel', 10)

    twist = Twist()
    twist.angular.z = 0.5  # Adjust rotation speed as needed

    duration = pi / abs(twist.angular.z)

    start_time = time.time()
    while time.time() - start_time < duration:
        cmd_vel_pub.publish(twist)
        rclpy.spin_once(navigator, timeout_sec=0.1)

    # Stop the robot after rotation
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    print('Custom action completed.')

if __name__ == '__main__':
    main()

