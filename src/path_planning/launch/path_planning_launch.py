import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_planner_config = os.path.join(
        get_package_share_directory('path_planning'),
        'config',
        'planner_params.yaml'
    )

    planner_config_la = DeclareLaunchArgument(
        'planner_config',
        default_value=default_planner_config,
        description='Path to the planner configuration file'
    )

    planner_node = Node(
        package='path_planning',
        executable='planner',   
        name='planner_node',
        output='screen',
        parameters=[LaunchConfiguration('planner_config')]
    )

    return LaunchDescription([
        planner_config_la,
        planner_node
    ])