import rclpy
import math
from reset_costmaps import deactivate_costmaps, activate_costmaps
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition



# Define destinations
destinations = {
    "pre_loading_position": [2.33, 0.18, (math.pi/2)],
    "loading_position": [2.36, -1.35, (math.pi/2)],
    "shipping_position": [-0.85, 1.5, (3*math.pi/2)],
    "initial_position": [-3.3, 0.18, (2*math.pi)]
}

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

def main():
    rclpy.init()

    # Initialize the navigator
    navigator = BasicNavigator()

    # Set the initial pose of the robot
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -3.3
    initial_pose.pose.position.y = 0.18
    initial_pose.pose.orientation.z = math.sin(math.pi)
    initial_pose.pose.orientation.w = math.cos(math.pi)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    # Create a ROS node
    nav_elevator_node = rclpy.create_node('navigation_and_elevator_controller')
    costmap_lifecycle_node = rclpy.create_node('costmap_lifecycle_manager')

    nav_elevator_node.declare_parameter('obstacle_layer.scan.obstacle_min_range')
    nav_elevator_node.declare_parameter('obstacle_layer.scan.raytrace_min_range')
    nav_elevator_node.declare_parameter('inflation_layer.inflation_radius')
    # Create the parameter clients for the costmap nodes once
    parameter_clients = {}
    costmap_nodes = [
        'local_costmap/local_costmap',
        'global_costmap/global_costmap'
    ]

    for costmap_node in costmap_nodes:
        client = nav_elevator_node.create_client(SetParameters, f'{costmap_node}/set_parameters')
        parameter_clients[costmap_node] = client
        if not client.wait_for_service(timeout_sec=5.0):
            nav_elevator_node.get_logger().error(f"Service {client.srv_name} not available for {costmap_node}")
            return

    # Define footprints
    unloaded_radius = 0.253
    fine_loading_radius = 0.08
    box_dimension = 0.22
    loaded_footprint = f'[[-{box_dimension}, -{box_dimension}], [-{box_dimension}, {box_dimension}], [{box_dimension}, {box_dimension}], [{box_dimension}, -{box_dimension}]]'
    large_radius = 0.58
    # Set initial footprint to unloaded radius
    success = set_footprint(nav_elevator_node, parameter_clients, footprint_type='radius', radius=unloaded_radius)
    if success:
        nav_elevator_node.get_logger().info("Footprint set to unloaded circular radius.")

    # Create publishers for elevator control
    elevator_up_publisher = nav_elevator_node.create_publisher(String, '/elevator_up', 10)
    elevator_down_publisher = nav_elevator_node.create_publisher(String, '/elevator_down', 10)

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
            rclpy.spin_once(nav_elevator_node, timeout_sec=0.1)

        # Check the result of the navigation task
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'Successfully reached {destination}.')

            if destination == "loading_position":
                print("Raising the elevator...")
                if deactivate_costmaps(costmap_lifecycle_node):
                    costmap_lifecycle_node.get_logger().info("Costmaps deactivated. Updating parameters...")

                    # Ensure set_costmap_parameters completes successfully before continuing
                    if set_costmap_parameters(nav_elevator_node, parameter_clients, raytrace_min=0.97, obstacle_min=0.97):
                        msg = String()
                        msg.data = 'up'
                        elevator_up_publisher.publish(msg)
                        set_inflation_radius(nav_elevator_node, parameter_clients, inflation_radius=0.6)
                        rclpy.spin_once(nav_elevator_node, timeout_sec=1.0)
                        success = set_footprint(nav_elevator_node, parameter_clients, footprint_type='polygon', polygon=loaded_footprint)
                        if success:
                            print("Footprint updated to loaded dimensions.")
                            if activate_costmaps(costmap_lifecycle_node):
                                print("Activating costmaps.")
                                current_obstacle_min = nav_elevator_node.get_parameter('local_costmap.obstacle_layer.scan.obstacle_min_range').value
                                current_raytrace_min = nav_elevator_node.get_parameter('local_costmap.obstacle_layer.scan.raytrace_min_range').value
                                nav_elevator_node.get_logger().info(f"Current obstacle_min_range: {current_obstacle_min}, raytrace_min_range: {current_raytrace_min}")
                            else:
                                print("failed to update costmaps")

                    else:
                        print("Failed to update costmap parameters. Aborting further actions.")
                


            elif destination == "shipping_position":
                # Publish to /elevator_down to lower the elevator
                print("Lowering the elevator...")
                success = set_footprint(nav_elevator_node, parameter_clients, footprint_type='radius', radius=fine_loading_radius)
                if success:
                    print("Footprint set to unloaded circular radius.")
                if set_costmap_parameters(nav_elevator_node, parameter_clients, raytrace_min=0.1, obstacle_min=0.1):
                    msg = String()
                    msg.data = 'down'
                    elevator_down_publisher.publish(msg)
                    set_inflation_radius(nav_elevator_node, parameter_clients, inflation_radius=0.25)
                    rclpy.spin_once(nav_elevator_node, timeout_sec=1.0)
            
            elif destination == "pre_loading_position":
                success = set_footprint(nav_elevator_node, parameter_clients, footprint_type='radius', radius=fine_loading_radius)
                if success:
                    print("Footprint set to unloaded circular radius.")
                #set_inflation_radius(node, parameter_clients, inflation_radius=0.05)
                rclpy.spin_once(nav_elevator_node, timeout_sec=1.0)
        
        elif result == TaskResult.CANCELED:
            print(f'Navigation to {destination} was canceled.')
            break

        elif result == TaskResult.FAILED:
            print(f'Failed to reach {destination}. Exiting.')
            exit(-1)

       

    print('Completed all destinations. Exiting.')
    nav_elevator_node.destroy_node()
    costmap_lifecycle_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






