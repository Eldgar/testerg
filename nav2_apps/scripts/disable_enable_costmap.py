import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

# Function to disable layers
def disable_costmaps(node, parameter_clients):
    layers_to_disable = [
        'obstacle_layer.enabled',
        'inflation_layer.enabled',
        'static_layer.enabled',
        'keepout_filter.enabled'
    ]
    success = True
    for costmap_node in parameter_clients:
        client = parameter_clients[costmap_node]
        req = SetParameters.Request()
        req.parameters = [Parameter(name=layer, value=False).to_parameter_msg() for layer in layers_to_disable]
        if client.wait_for_service(timeout_sec=5.0):
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future)
            if future.result() is not None and all(result.successful for result in future.result().results):
                node.get_logger().info(f"Successfully disabled layers for {costmap_node}")
            else:
                node.get_logger().error(f"Failed to disable layers for {costmap_node}")
                success = False
        else:
            node.get_logger().error(f"Service {costmap_node}/set_parameters not available")
            success = False
    return success

# Function to enable layers
def enable_costmaps(node, parameter_clients):
    layers_to_enable = [
        'obstacle_layer.enabled',
        'inflation_layer.enabled',
        'static_layer.enabled',
        'keepout_filter.enabled'
    ]
    success = True
    for costmap_node in parameter_clients:
        client = parameter_clients[costmap_node]
        req = SetParameters.Request()
        req.parameters = [Parameter(name=layer, value=True).to_parameter_msg() for layer in layers_to_enable]
        if client.wait_for_service(timeout_sec=5.0):
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future)
            if future.result() is not None and all(result.successful for result in future.result().results):
                node.get_logger().info(f"Successfully enabled layers for {costmap_node}")
            else:
                node.get_logger().error(f"Failed to enable layers for {costmap_node}")
                success = False
        else:
            node.get_logger().error(f"Service {costmap_node}/set_parameters not available")
            success = False
    return success



