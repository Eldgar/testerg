import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


# Define destinations
destinations = {
    "pre_loading_position": [3.59, -0.4, (-math.pi/2)],
    "loading_position": [3.6, 1.35, (math.pi/2)],
    "shipping_position": [-0.85, 1.5, (3*math.pi/2)],
    "initial_position": [-3.3, 0.18, (2*math.pi)]
}


def set_costmap_parameters(node, raytrace_min, obstacle_min):
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

    # Define the correct node paths for local and global costmaps
    costmap_nodes = [
        "/local_costmap/local_costmap",
        "/global_costmap/global_costmap"
    ]

    # Create parameter clients for each costmap node
    for costmap_node in costmap_nodes:
        client = node.create_client(SetParameters, f'{costmap_node}/set_parameters')
        
        # Wait for the service to become available
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error(f"Service {costmap_node}/set_parameters is not available.")
            continue

        # Send parameter update request
        request = SetParameters.Request(parameters=parameters)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            results = future.result().results
            if all([res.successful for res in results]):
                node.get_logger().info(f"Costmap parameters successfully updated on {costmap_node}")
            else:
                node.get_logger().error(f"Failed to update costmap parameters on {costmap_node}")
        else:
            node.get_logger().error(f"Failed to call service {costmap_node}/set_parameters")

    return True



def set_inflation_radius(node, parameter_clients, inflation_radius):
    parameters = [
        Parameter(
            name='inflation_layer.inflation_radius',  # Adjust to the correct namespace
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
            Parameter(
                name='footprint',
                value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value='[]')
            )
        ]
    elif footprint_type == 'polygon' and polygon is not None:
        parameters = [
            Parameter(
                name='footprint',
                value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=polygon)
            ),
            Parameter(
                name='robot_radius',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.0)
            )
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

def main():
    rclpy.init()

    # Initialize the navigator
    navigator = BasicNavigator()

    # Set the initial pose of the robot
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.47
    initial_pose.pose.position.y = -0.4
    initial_pose.pose.orientation.z = math.sin(0.99*math.pi)
    initial_pose.pose.orientation.w = math.cos(0.99*math.pi)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    # Create a ROS node
    node = rclpy.create_node('navigation_and_elevator_controller')

    # Create the parameter clients for the costmap nodes once
    parameter_clients = {}
    costmap_nodes = [
        'local_costmap/local_costmap',
        'global_costmap/global_costmap'
    ]

    for costmap_node in costmap_nodes:
        client = node.create_client(SetParameters, f'{costmap_node}/set_parameters')
        parameter_clients[costmap_node] = client
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error(f"Service {client.srv_name} not available for {costmap_node}")
            return

    # Define footprints
    unloaded_radius = 0.253
    fine_loading_radius = 0.08
    loaded_footprint = '[[-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5]]'
    large_radius = 0.58
    # Set initial footprint to unloaded radius
    success = set_footprint(node, parameter_clients, footprint_type='radius', radius=unloaded_radius)
    if success:
        node.get_logger().info("Footprint set to unloaded circular radius.")

    # Create publishers for elevator control
    elevator_up_publisher = node.create_publisher(String, '/elevator_up', 10)
    elevator_down_publisher = node.create_publisher(String, '/elevator_down', 10)

    # Sequence of destinations to navigate through
    destination_sequence = ['pre_loading_position', 'loading_position', 'shipping_position', 'initial_position']

    for destination in destination_sequence:

        # Set up the pose for the current destination
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.header.stamp = navigator.get_clock().now().to_msg()
        target_pose.pose.position.x = destinations[destination][0]
        target_pose.pose.position.y = destinations[destination][1]
        target_pose.pose.orientation.z = math.sin(destinations[destination][2]/2)
        target_pose.pose.orientation.w = math.cos(destinations[destination][2]/2)

        print(f'Navigating to {destination}...')
        navigator.goToPose(target_pose)

        # Wait until the navigation task is complete
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            rclpy.spin_once(node, timeout_sec=0.1)

        # Check the result of the navigation task
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'Successfully reached {destination}.')

            if destination == "loading_position":
                # Publish to /elevator_up to raise the elevator
                print("Raising the elevator...")
                #success = set_footprint(node, parameter_clients, footprint_type='footprint', polygon=loaded_footprint)
                if success:
                    print("Footprint updated to loaded dimensions.")
                set_inflation_radius(node, parameter_clients, inflation_radius=0.38)
                set_costmap_parameters(node, raytrace_min=0.5, obstacle_min=0.5)
                msg = String()
                msg.data = 'up'
                elevator_up_publisher.publish(msg)
                rclpy.spin_once(node, timeout_sec=1.0) 

            elif destination == "shipping_position":
                # Publish to /elevator_down to lower the elevator
                print("Lowering the elevator...")
                success = set_footprint(node, parameter_clients, footprint_type='radius', radius=unloaded_radius)
                if success:
                    print("Footprint set to unloaded circular radius.")
                set_costmap_parameters(node, raytrace_min=0.15, obstacle_min=0.15)
                msg = String()
                msg.data = 'down'
                elevator_down_publisher.publish(msg)
                rclpy.spin_once(node, timeout_sec=1.0)
            
            elif destination == "pre_loading_position":
                #success = set_footprint(node, parameter_clients, footprint_type='radius', radius=fine_loading_radius)
                if success:
                    print("Footprint set to unloaded circular radius.")
                set_inflation_radius(node, parameter_clients, inflation_radius=0.05)
                rclpy.spin_once(node, timeout_sec=1.0)



        elif result == TaskResult.CANCELED:
            print(f'Navigation to {destination} was canceled.')
            break

        elif result == TaskResult.FAILED:
            print(f'Failed to reach {destination}. Exiting.')
            exit(-1)

    print('Completed all destinations. Exiting.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


