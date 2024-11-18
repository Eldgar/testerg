#ros2 launch path_planner_server pathplanner_real.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Paths to config files
    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller_real.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator_real.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server_real.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery_real.yaml')
    filters_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'filters_real.yaml')
    waypoint_follower_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'waypoint_follower_real.yaml')
    #rviz_config = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'pathplanning_rviz_config.rviz')

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic name for cmd_vel')

    # Get the launch configuration
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    return LaunchDescription([
        # Launch controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml],
            #remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]
        ),
        
        # Launch planner server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]
        ),
        
        Node(
              package='nav2_map_server',
              executable='map_server',
              name='filter_mask_server',
              output='screen',
              emulate_tty=True,
              parameters=[filters_yaml]),
        
        Node(
             package='nav2_map_server',
             executable='costmap_filter_info_server',
             name='costmap_filter_info_server',
             output='screen',
             emulate_tty=True,
             parameters=[filters_yaml]),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[recovery_yaml],
            output='screen'),


        # Launch behavior tree navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[waypoint_follower_yaml]),

        cmd_vel_topic_arg,
        Node(
            package='attach_shelf', 
            executable='move_to_frame_server',
            name='move_to_frame_server',
            output='screen',
            parameters=[{'use_sim_time': False}, {'cmd_vel_topic': cmd_vel_topic}]
        ),
        
        # Launch lifecycle manager for path planner
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': [
                                        #'map_server',
                                        #'amcl',
                                        'planner_server',
                                        'controller_server',
                                        'waypoint_follower',
                                        'behavior_server',
                                        'bt_navigator',
                                        'filter_mask_server',
                                        'costmap_filter_info_server'
                                        ]}]
        ),
    ])
