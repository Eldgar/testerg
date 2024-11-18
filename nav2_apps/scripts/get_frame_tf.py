import math
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time


def get_frame_coordinates(node, tf_buffer, frame='robot_base_footprint'):
    try:
        now = node.get_clock().now()
        # Wait for the transform to become available
        tf_buffer.can_transform('map', frame, now, timeout=Duration(seconds=1.0))
        # Lookup the latest available transform
        trans = tf_buffer.lookup_transform('map', frame, Time())
        # Proceed with your calculations
        orientation_z = trans.transform.rotation.z
        orientation_w = trans.transform.rotation.w
        orientation = math.atan2(2.0 * orientation_w * orientation_z, 1.0 - 2.0 * orientation_z**2)
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        #node.get_logger().info(f"Retrieved {frame} coordinates: x={x}, y={y}, z={orientation_z}, w={orientation_w}")
        return [x, y, orientation]
    except Exception as e:
        node.get_logger().error(f"Error fetching transform: {e}")
        return None