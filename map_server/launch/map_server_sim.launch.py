import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the 'map_file' argument, with a default value
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='Name of the map file to load')

    # Path to the RViz configuration file
    rviz_config_file = os.path.join(
        get_package_share_directory('map_server'), 'config', 'mapper_rviz_config.rviz')

    # Get the map file from the launch argument
    map_file_name = LaunchConfiguration('map_file')

    # Use LaunchConfiguration substitutions directly in the node parameters
    map_file_path = [os.path.join(get_package_share_directory('map_server'), 'config'), '/', map_file_name]

    
    return LaunchDescription([
        # Declare the map file argument
        declare_map_file_cmd,

        # Start the map_server node to publish the map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename': map_file_path}]
        ),

        # Start RViz2 to visualize the map
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_file]
        ),


        # Add a static transform publisher to link 'map' and 'odom'
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_broadcaster',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': True}]
        ),
    ])


