from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_pure_pursuit_config = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'pure_pursuit_params.yaml'
    )

    pure_pursuit_la = DeclareLaunchArgument(
        'pure_pursuit_config',
        default_value=default_pure_pursuit_config,
        description='Path to the pure pursuit configuration file'
    )

    ld = LaunchDescription([pure_pursuit_la])

    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node',
        name='pure_pursuit',
        output='screen',
        parameters=[LaunchConfiguration('pure_pursuit_config')]
    )

    ld.add_action(pure_pursuit_node)

    return ld
