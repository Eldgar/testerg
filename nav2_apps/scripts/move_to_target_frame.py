import math
import rclpy
import time
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer



def get_robot_base_coordinates(node, tf_buffer, tf_listener):
    try:
        tf_listener = tf_listener
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


def get_target_coordinates(node, tf_buffer, tf_listener, target):
    try:
        tf_listener = tf_listener
        trans = tf_buffer.lookup_transform('map', target, rclpy.time.Time())
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        # Convert quaternion to Euler angles
        q = trans.transform.rotation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        node.get_logger().info(f"Retrieved {target} coordinates: x={x}, y={y}, yaw={yaw}")
        return [x, y, yaw]
    except Exception as e:
        node.get_logger().error(f"Error fetching transform: {e}")
        return None


def move_to_target(nav_node, tf_buffer, tf_listener, target, position_tolerance=0.05, angle_tolerance=0.05, timeout=45):
    nav_node.get_logger().info(f"Moving to position {target}")
    cmd_vel_publisher = nav_node.create_publisher(Twist, '/cmd_vel', 10)
    twist = Twist()
    time.sleep(2)
    start_time = time.time()
    
    # Phase 1: Move to target position
    while rclpy.ok() and (time.time() - start_time) < timeout:
        # Get current robot coordinates
        robot_coordinates = get_robot_base_coordinates(nav_node, tf_buffer, tf_listener)
        if robot_coordinates is None:
            nav_node.get_logger().warn("Unable to get robot coordinates")
            time.sleep(0.1)
            continue
        current_x, current_y, current_orientation = robot_coordinates
        target_x, target_y, target_orientation = get_target_coordinates(nav_node, tf_buffer, tf_listener, target)
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

        if distance <= position_tolerance:
            nav_node.get_logger().info("Reached the target position!")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            cmd_vel_publisher.publish(twist)
            break  # Proceed to orientation adjustment

        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        angle_diff = angle_to_target - current_orientation
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        twist.linear.x = min(0.03, distance)
        twist.angular.z = min(0.2, max(-0.2, angle_diff))
        cmd_vel_publisher.publish(twist)
        time.sleep(0.1)
    
    # Phase 2: Adjust orientation
    nav_node.get_logger().info("Adjusting orientation...")
    while rclpy.ok() and (time.time() - start_time) < timeout:
        # Get current robot orientation
        robot_coordinates = get_robot_base_coordinates(nav_node, tf_buffer)
        target_coordinates = get_target_coordinates(nav_node, tf_buffer, target)
        if robot_coordinates is None:
            nav_node.get_logger().warn("Unable to get robot coordinates")
            time.sleep(0.1)
            continue
        current_orientation = robot_coordinates[2]
        target_orientation = target_coordinates[2]
        angle_diff = target_orientation - current_orientation
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        if abs(angle_diff) <= angle_tolerance:
            nav_node.get_logger().info("Orientation adjusted!")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            cmd_vel_publisher.publish(twist)
            break

        twist.linear.x = 0.0
        twist.angular.z = min(0.5, max(-0.5, angle_diff))
        cmd_vel_publisher.publish(twist)
        time.sleep(0.1)
    
    # Ensure the robot stops moving
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_vel_publisher.publish(twist)
    return True



