import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_cartographer(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    pkg_share = get_package_share_directory('cartographer_slam')

    # Define the paths to the launch files
    cartographer_sim_launch = os.path.join(pkg_share, 'launch', 'cartographer_sim.launch.py')
    cartographer_real_launch = os.path.join(pkg_share, 'launch', 'cartographer_real.launch.py')

    # Conditionally include the correct launch file
    if use_sim_time == 'true':
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(cartographer_sim_launch))]
    else:
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(cartographer_real_launch))]


def generate_launch_description():
    # Declare the launch argument for use_sim_time
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        OpaqueFunction(function=launch_cartographer),
    ])

