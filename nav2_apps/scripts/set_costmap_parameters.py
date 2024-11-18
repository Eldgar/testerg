import rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

def set_inflation_radius(node, parameter_clients, inflation_radius):
    parameters = [
        Parameter(
            name='inflation_layer.inflation_radius',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=inflation_radius)
        )
    ]
    
    # Send parameter update requests to each costmap node
    for costmap_node, client in parameter_clients.items():
        request = SetParameters.Request(parameters=parameters)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            results = future.result().results
            if all([res.successful for res in results]):
                node.get_logger().info(f"Inflation radius successfully updated on {costmap_node} to {inflation_radius}")
            else:
                node.get_logger().error(f"Failed to update inflation radius on {costmap_node}")
        else:
            node.get_logger().error(f"Failed to call service {client.srv_name} on {costmap_node}")
    return True

def set_costmap_parameters(node, parameter_clients, raytrace_min, obstacle_min):

    parameters = [
        Parameter(
            name='obstacle_layer.scan.obstacle_min_range',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=obstacle_min)
        ),
        Parameter(
            name='obstacle_layer.scan.raytrace_min_range',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=raytrace_min)
        ),
    ]

    for costmap_node, client in parameter_clients.items():
        # Ensure the service is available
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error(f"Service {costmap_node}/set_parameters is not available.")
            return False  # Return False if the service is unavailable

        # Send parameter update request
        request = SetParameters.Request(parameters=parameters)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            results = future.result().results
            if all(res.successful for res in results):
                node.get_logger().info(f"Costmap parameters successfully updated on {costmap_node}")
            else:
                node.get_logger().error(f"Failed to update costmap parameters on {costmap_node}")
                return False  # Return False if any parameter update fails
        else:
            node.get_logger().error(f"Failed to call service {costmap_node}/set_parameters")
            return False  # Return False if the service call itself fails

    return True


def set_footprint(node, parameter_clients, footprint_type, radius=None, polygon=None):
    # Prepare the parameter changes
    parameters = []
    if footprint_type == 'radius' and radius is not None:
        # Set robot_radius and clear footprint
        parameters = [
            Parameter(
                name='robot_radius',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=radius)
            ),
        ]
    elif footprint_type == 'polygon' and polygon is not None:
        parameters = [
            Parameter(
                name='footprint',
                value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=polygon)
            ),
        ]
    else:
        node.get_logger().error("Invalid footprint_type or missing parameters.")
        return False

    # Send parameter update requests to each costmap node
    for costmap_node, client in parameter_clients.items():
        request = SetParameters.Request()
        request.parameters = parameters
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            results = future.result().results
            if all([res.successful for res in results]):
                node.get_logger().info(f"Parameters successfully updated on {costmap_node}.")
            else:
                node.get_logger().error(f"Failed to update parameters on {costmap_node}.")
        else:
            node.get_logger().error(f"Failed to call service {client.srv_name} on {costmap_node}.")
            return False
    return True

def set_max_velocity(node, client, max_vel_x=0.1, max_vel_theta=0.23):
    req = SetParameters.Request()
    req.parameters = [
        Parameter(name='FollowPath.max_vel_x', value=ParameterValue(type=3, double_value=max_vel_x)),
        Parameter(name='FollowPath.max_vel_theta', value=ParameterValue(type=3, double_value=max_vel_theta))
    ]

    # Send the request
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    # Check the result
    if future.result() is not None and all(result.successful for result in future.result().results):
        print("Successfully set max_vel_x to 0.1 and max_vel_theta to 0.2")
    else:
        print("Failed to set max velocity parameters.")

def set_goal_tolerances(node, parameter_clients, xy_tolerance, yaw_tolerance):
    # Define the parameters to update
    parameters = [
        Parameter(
            name='general_goal_checker.xy_goal_tolerance',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=xy_tolerance)
        ),
        Parameter(
            name='general_goal_checker.yaw_goal_tolerance',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=yaw_tolerance)
        )
    ]
    
    # Send parameter update requests to each costmap node
    for costmap_node, client in parameter_clients.items():
        request = SetParameters.Request(parameters=parameters)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            results = future.result().results
            if all([res.successful for res in results]):
                node.get_logger().info(f"Tolerances successfully updated on {costmap_node}: xy_goal_tolerance={xy_tolerance}, yaw_goal_tolerance={yaw_tolerance}")
            else:
                node.get_logger().error(f"Failed to update tolerances on {costmap_node}")
        else:
            node.get_logger().error(f"Failed to call service {client.srv_name} on {costmap_node}")
    return True
