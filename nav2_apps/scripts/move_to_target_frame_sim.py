import math
import rclpy
import time
from geometry_msgs.msg import Twist
from get_frame_tf import get_frame_coordinates
from get_crate_tf import get_robot_cart_laser_coordinates
from tf2_ros import TransformListener, Buffer


def get_robot_base_coordinates(node, tf_buffer):
    try:
        trans = tf_buffer.lookup_transform('map', 'robot_base_footprint', rclpy.time.Time())
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        # Convert quaternion to Euler angles
        q = trans.transform.rotation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        node.get_logger().info(f"Retrieved robot_base_footprint coordinates: x={x}, y={y}, yaw={yaw}")
        return [x, y, yaw]
    except Exception as e:
        node.get_logger().error(f"Error fetching transform: {e}")
        return None

def get_target_coordinates(node, tf_buffer, target):
    try:
        trans = tf_buffer.lookup_transform(
            'map',
            target,
            timeout=rclpy.duration.Duration(seconds=1.0))
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        # Convert quaternion to Euler angles
        q = trans.transform.rotation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return [x, y, yaw]
    except Exception as e:
        node.get_logger().error(f"Error fetching transform: {e}")
        return None



def move_to_target(nav_node, tf_buffer, target, position_tolerance=0.05):
    # Get current robot coordinates
    robot_coordinates = get_robot_base_coordinates(nav_node, tf_buffer)
    if robot_coordinates is None:
        nav_node.get_logger().warn("Unable to get robot coordinates")
        return False
    current_x, current_y, current_orientation = robot_coordinates

    target_coordinates = get_target_coordinates(nav_node, tf_buffer, target)
    if target_coordinates is None:
        nav_node.get_logger().warn("Unable to get target coordinates")
        return False
    target_x, target_y, target_orientation = target_coordinates

    # Calculate distance and angle to target
    distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
    angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
    angle_diff = angle_to_target - current_orientation
    angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

    cmd_vel_publisher = nav_node.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
    twist = Twist()

    if distance <= position_tolerance:
        nav_node.get_logger().info("Reached the target position!")
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        cmd_vel_publisher.publish(twist)
        return True  # Target reached

    # Compute velocities
    twist.linear.x = min(0.12, distance / 4)
    twist.angular.z = max(-0.5, min(0.5, angle_diff / 3))
    cmd_vel_publisher.publish(twist)
    nav_node.get_logger().info(f"Publishing cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}")

    return False  # Target not yet reached


def adjust_orientation(nav_node, tf_buffer, target, angle_tolerance=0.10):
    # Get current robot orientation
    robot_coordinates = get_robot_base_coordinates(nav_node, tf_buffer)
    if robot_coordinates is None:
        nav_node.get_logger().warn("Unable to get robot coordinates")
        return False
    _, _, current_orientation = robot_coordinates

    target_coordinates = get_target_coordinates(nav_node, tf_buffer, target)
    if target_coordinates is None:
        nav_node.get_logger().warn("Unable to get target coordinates")
        return False
    _, _, target_orientation = target_coordinates

    # Calculate angle difference
    angle_diff = target_orientation - current_orientation
    angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

    cmd_vel_publisher = nav_node.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
    twist = Twist()

    if abs(angle_diff) <= angle_tolerance:
        nav_node.get_logger().info("Orientation adjusted!")
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        cmd_vel_publisher.publish(twist)
        return True  # Orientation adjusted

    # Compute angular velocity
    twist.linear.x = 0.0
    twist.angular.z = max(-0.5, min(0.5, angle_diff))
    cmd_vel_publisher.publish(twist)
    nav_node.get_logger().info(f"Adjusting orientation: angular.z={twist.angular.z}")

    return False  # Orientation not yet adjusted


