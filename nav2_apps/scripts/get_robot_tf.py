import math
import rclpy

def get_robot_base_coordinates(node, tf_buffer):
    try:
        trans = tf_buffer.lookup_transform('map', 'robot_base_footprint', rclpy.time.Time())
        orientation_z = trans.transform.rotation.z
        orientation_w = trans.transform.rotation.w
        orientation = math.atan2(2.0 * orientation_w * orientation_z, 1.0 - 2.0 * orientation_z**2)
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        orientation_z = trans.transform.rotation.z
        orientation_w = trans.transform.rotation.w
        #node.get_logger().info(f"Retrieved robot_base_footprint coordinates: x={x}, y={y}, z={orientation_z}, w={orientation_w}")
        return [x, y, orientation]
    except Exception as e:
        #node.get_logger().error(f"Error fetching transform: {e}")
        return None