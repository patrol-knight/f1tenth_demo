import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    default_path_planning_config = os.path.join(
        get_package_share_directory('path_planning'),
        'config',
        'path_planning_param.yaml'
    )

    path_planning_la = DeclareLaunchArgument(
        'path_planning_config',
        default_value=default_path_planning_config,
        description='Path to the path planning configuration file'
    )

    ld = LaunchDescription([path_planning_la])

    map_inflator_node = Node(
        package="path_planning",
        executable="map_inflator",
        name="map_inflator_node",
        output="screen",
        parameters=[LaunchConfiguration('path_planning_config')]
    )

    path_planning_node = Node(
        package="path_planning",                 # <-- your ROS2 package name
        executable="planner_nn_2opt",            # <-- your console_scripts entry
        name="path_planning_nn_2opt_node",            # optional override
        output="screen",
        parameters=[LaunchConfiguration('path_planning_config')]
    )

    smoothing_node = Node(
        package="path_planning",
        executable="smoothing",
        name="smoothing_node",
        output="screen",
        parameters=[LaunchConfiguration('path_planning_config')]
    )

    ld.add_action(map_inflator_node)
    ld.add_action(path_planning_node)
    ld.add_action(smoothing_node)

    return ld
