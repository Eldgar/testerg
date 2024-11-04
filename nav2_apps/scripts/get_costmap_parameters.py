import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters


def get_costmap_parameters(node, parameter_clients, parameter_names):
    parameters_values = {}
    for costmap_node, clients in parameter_clients.items():
        get_client = clients['get']
        # Prepare the request
        request = GetParameters.Request(names=parameter_names)
        # Call the service
        future = get_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            results = future.result().values
            parameters_values[costmap_node] = {}
            for name, value in zip(parameter_names, results):
                if value.type == ParameterType.PARAMETER_DOUBLE:
                    parameters_values[costmap_node][name] = value.double_value
                elif value.type == ParameterType.PARAMETER_INTEGER:
                    parameters_values[costmap_node][name] = value.integer_value
                elif value.type == ParameterType.PARAMETER_STRING:
                    parameters_values[costmap_node][name] = value.string_value
                else:
                    parameters_values[costmap_node][name] = None
        else:
            node.get_logger().error(f"Failed to get parameters from {costmap_node}")
    return parameters_values
