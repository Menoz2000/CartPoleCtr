from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():

    config_node = os.path.join(
        get_package_share_directory('force_simulator'),
        'config',
        'force_simulator_conf.yaml'
    )

    node=Node(
        package='force_simulator',
        name='force_simulator_node',
        executable='force_simulator',
        parameters=[
            config_node,
        ]
    )

    return LaunchDescription([
        node
    ])