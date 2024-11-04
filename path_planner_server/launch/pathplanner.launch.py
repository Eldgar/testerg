import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the 'map_file' argument
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_real.yaml',
        description='Name of the map file to load (determines which launch file to use)'
    )

    # Define launch configurations
    map_file = LaunchConfiguration('map_file')

    # Paths to the launch files
    pkg_share = get_package_share_directory('path_planner_server')
    pathplanner_sim_launch = os.path.join(pkg_share, 'launch', 'pathplanner_sim.launch.py')
    pathplanner_real_launch = os.path.join(pkg_share, 'launch', 'pathplanner_real.launch.py')

    # Function to select the appropriate launch file based on 'map_file'
    def include_pathplanner_launch(context):
        # Get the actual value of 'map_file' at runtime
        map_file_value = map_file.perform(context)

        # Decide which launch file to include
        if map_file_value == 'warehouse_map_real.yaml':
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(pathplanner_real_launch)
            )]
        else:
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(pathplanner_sim_launch)
            )]

    # Use OpaqueFunction to perform the conditional logic at launch time
    conditional_inclusion = OpaqueFunction(function=include_pathplanner_launch)

    return LaunchDescription([
        declare_map_file_cmd,
        conditional_inclusion,
    ])