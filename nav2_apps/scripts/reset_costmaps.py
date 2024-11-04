import rclpy
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

def change_state(node, client, transition_id):
    request = ChangeState.Request()
    request.transition.id = transition_id
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None and future.result().success:
        node.get_logger().info(f"Transition {transition_id} successful.")
    else:
        node.get_logger().error(f"Transition {transition_id} failed.")

def deactivate_costmaps(node):
    # Deactivate both local and global costmaps
    local_costmap_client = node.create_client(ChangeState, 'local_costmap/local_costmap/change_state')
    global_costmap_client = node.create_client(ChangeState, 'global_costmap/global_costmap/change_state')
    
    # Wait for the services to become available
    if not local_costmap_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Local costmap service not available.")
        return False
    if not global_costmap_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Global costmap service not available.")
        return False

    # Transition IDs: 3 for 'deactivate'
    change_state(node, local_costmap_client, Transition.TRANSITION_DEACTIVATE)
    change_state(node, global_costmap_client, Transition.TRANSITION_DEACTIVATE)
    return True

def activate_costmaps(node):
    # Reactivate both local and global costmaps
    local_costmap_client = node.create_client(ChangeState, 'local_costmap/local_costmap/change_state')
    global_costmap_client = node.create_client(ChangeState, 'global_costmap/global_costmap/change_state')
    
    # Wait for the services to become available
    if not local_costmap_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Local costmap service not available.")
        return False
    if not global_costmap_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Global costmap service not available.")
        return False

    # Transition IDs: 1 for 'activate'
    change_state(node, local_costmap_client, Transition.TRANSITION_ACTIVATE)
    change_state(node, global_costmap_client, Transition.TRANSITION_ACTIVATE)
    return True