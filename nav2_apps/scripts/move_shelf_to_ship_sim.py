#python3 ~/ros2_ws/src/warehouse_project/nav2_apps/scripts/move_shelf_to_ship_sim.py
import rclpy
import math
import time
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from tf2_ros import TransformListener, Buffer
from attach_shelf.srv import MoveToFrame

from disable_enable_costmap import enable_costmaps, disable_costmaps
from get_costmap_parameters import get_costmap_parameters
from set_costmap_parameters import set_inflation_radius, set_footprint
# Define destinations
destinations = {
    "initial_position": [-3.3, 0.18, (2 * math.pi)],
    "pre_loading_position": [2.27, 0.2, (math.pi / 2)],
    "loading_position": [2.36, -1.35, (math.pi / 2)],
    "pre_shipping_position": [-0.87, 0.2, (3 * math.pi / 2)],
    "shipping_position": [-0.85, 1.5, (3 * math.pi / 2)],
    "final_position": [-3.3, 0.18, (2 * math.pi)]
}

def main():
    rclpy.init()

    # Initialize the navigator
    navigator = BasicNavigator()

    # Set the initial pose of the robot
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = destinations["initial_position"][0]
    initial_pose.pose.position.y = destinations["initial_position"][1]
    initial_pose.pose.orientation.z = math.sin(destinations["initial_position"][2] / 2)
    initial_pose.pose.orientation.w = math.cos(destinations["initial_position"][2] / 2)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    # Create a ROS node
    nav_elevator_node = rclpy.create_node('navigation_and_elevator_controller')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, nav_elevator_node)
    # Create the parameter clients for the costmap nodes
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
    unloaded_radius = 0.25
    box_dimension = 0.35
    loaded_footprint = f'[[-{box_dimension}, -{box_dimension}], [-{box_dimension}, {box_dimension}], [{box_dimension}, {box_dimension}], [{box_dimension}, -{box_dimension}]]'

    # Set initial footprint to unloaded radius
    success = set_footprint(nav_elevator_node, parameter_clients, footprint_type='radius', radius=unloaded_radius)
    if success:
        nav_elevator_node.get_logger().info("Footprint set to unloaded circular radius.")

    # Create publishers for elevator control
    elevator_up_publisher = nav_elevator_node.create_publisher(String, '/elevator_up', 10)
    elevator_down_publisher = nav_elevator_node.create_publisher(String, '/elevator_down', 10)

    # Create the navigation sequence
    navigation_sequence = [
        {
            "type": "waypoints",
            "points": ["pre_loading_position"]
        },
        {
            "type": "goal_pose",
            "destination": "loading_position"
        },
        {
            "type": "waypoints",
            "points": ["pre_shipping_position"]
        },
        {
            "type": "goal_pose",
            "destination": "shipping_position"
        },
        {
            "type": "waypoints",
            "points": ["pre_shipping_position"]
        },
        {
            "type": "goal_pose",
            "destination": "final_position"
        }
    ]
    for nav_point in navigation_sequence:
        
        if nav_point["type"] == "waypoints":
            # Create a list of waypoints
            waypoints = []
            for waypoint_name in nav_point["points"]:
                coords = destinations[waypoint_name]
                waypoint = PoseStamped()
                waypoint.header.frame_id = 'map'
                waypoint.header.stamp = navigator.get_clock().now().to_msg()
                waypoint.pose.position.x = coords[0]
                waypoint.pose.position.y = coords[1]
                # Default orientation (since orientation doesn't matter)
                waypoint.pose.orientation.z = math.sin(coords[2] / 2)
                waypoint.pose.orientation.w = math.cos(coords[2] / 2)
                waypoints.append(waypoint)

            print(f'Navigating through waypoints: {nav_point["points"]}')
            navigator.followWaypoints(waypoints)

            # Wait until the navigation task is complete
            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                rclpy.spin_once(nav_elevator_node, timeout_sec=0.1)


            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Successfully reached waypoints.')
                if waypoint_name == "pre_loading_position":
                    # target_frame = "map"  # Replace with the desired target frame

                    # # Wait for the /move_to_frame service to be available
                    # move_to_frame_client = nav_elevator_node.create_client(MoveToFrame, "/move_to_frame")
                    # if not move_to_frame_client.wait_for_service(timeout_sec=5.0):
                    #     print("Service /move_to_frame not available.")
                    #     exit(-1)

                    # # Call the /move_to_frame service
                    # request = MoveToFrame.Request()
                    # request.target_frame = target_frame

                    # future = move_to_frame_client.call_async(request)

                    # # Wait for the service response
                    # while rclpy.ok():
                    #     rclpy.spin_once(nav_elevator_node, timeout_sec=0.1)
                    #     if future.done():
                    #         try:
                    #             response = future.result()
                    #             if response.success:
                    #                 print(f"Successfully moved to the target frame: {target_frame}.")
                    #             else:
                    #                 print(f"Failed to move to the target frame: {target_frame}. Reason: {response.message}")
                    #             break
                    #         except Exception as e:
                    #             print(f"Service call failed: {e}")
                    #             exit(-1)
                    disable_costmaps(nav_elevator_node, parameter_clients)
                    
                    set_inflation_radius(nav_elevator_node, parameter_clients, inflation_radius=0.001)
                    set_footprint(nav_elevator_node, parameter_clients, footprint_type='radius', radius=0.003)
                    print('changing radius to enter crate')
            else:
                print('Failed to navigate through waypoints.')
                exit(-1)

        elif nav_point["type"] == "goal_pose":
            destination = nav_point["destination"]
            coords = destinations[destination]

            # Set up the pose for the current destination
            target_pose = PoseStamped()
            target_pose.header.frame_id = 'map'
            target_pose.header.stamp = navigator.get_clock().now().to_msg()
            target_pose.pose.position.x = coords[0]
            target_pose.pose.position.y = coords[1]
            target_pose.pose.orientation.z = math.sin(coords[2] / 2)
            target_pose.pose.orientation.w = math.cos(coords[2] / 2)

            print(f'Navigating to {destination}...')
            navigator.goToPose(target_pose)

            # Wait until the navigation task is complete
            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                rclpy.spin_once(nav_elevator_node, timeout_sec=0.2)

            # Check the result of the navigation task
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print(f'Successfully reached {destination}.')

                if destination == "loading_position":
                    print("Raising the elevator...")
                    # Adjust footprints and inflation radius
                    enable_costmaps(nav_elevator_node, parameter_clients)
                    success = set_footprint(nav_elevator_node, parameter_clients, footprint_type='polygon', polygon=loaded_footprint)
                    set_inflation_radius(nav_elevator_node, parameter_clients, inflation_radius=0.37)
                    msg = String()
                    msg.data = 'up'
                    elevator_up_publisher.publish(msg)
                    if success:
                        print("Footprint updated to loaded dimensions.")
                    else:
                        print("Failed to update costmaps.")

                    cmd_vel_publisher = nav_elevator_node.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
                    forward_cmd = Twist()
                    forward_cmd.linear.x = 0.1
                    print("Moving forward for 5 seconds...")
                    start_time = nav_elevator_node.get_clock().now().nanoseconds / 1e9
                    while (nav_elevator_node.get_clock().now().nanoseconds / 1e9) - start_time < 5.0:
                        cmd_vel_publisher.publish(forward_cmd)
                        time.sleep(0.1)

                    forward_cmd.linear.x = 0.0
                    cmd_vel_publisher.publish(forward_cmd)
                    print("Stopped moving forward.")

                    
                elif destination == "shipping_position":
                    print("Lowering the elevator...")
                    success = set_footprint(nav_elevator_node, parameter_clients, footprint_type='radius', radius=unloaded_radius)
                    set_inflation_radius(nav_elevator_node, parameter_clients, inflation_radius=0.15)
                    msg = String()
                    msg.data = 'down'
                    elevator_down_publisher.publish(msg)
                    if success:
                        print("Footprint set to unloaded circular radius.")
                    else:
                        print("Failed to update costmaps.")
                    cmd_vel_publisher = nav_elevator_node.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
                    forward_cmd = Twist()
                    forward_cmd.linear.x = 0.1
                    print("Moving forward for 10 seconds...")
                    start_time = nav_elevator_node.get_clock().now().nanoseconds / 1e9
                    while (nav_elevator_node.get_clock().now().nanoseconds / 1e9) - start_time < 3.0:
                        cmd_vel_publisher.publish(forward_cmd)
                        time.sleep(0.1)

                    forward_cmd.linear.x = 0.0
                    cmd_vel_publisher.publish(forward_cmd)
                    print("Stopped moving forward.")

            elif result == TaskResult.CANCELED:
                print(f'Navigation to {destination} was canceled.')
                break

            elif result == TaskResult.FAILED:
                print(f'Failed to reach {destination}. Exiting.')
                exit(-1)

    print('Completed all destinations. Exiting.')
    nav_elevator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()







