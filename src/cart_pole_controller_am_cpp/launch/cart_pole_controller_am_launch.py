from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():

    config_node = os.path.join(
        get_package_share_directory('cart_pole_controller_am_cpp'),
        'config',
        'cart_pole_controller_am_conf.yaml'
    )

    node=Node(
        package='cart_pole_controller_am_cpp',
        name='cart_pole_controller',
        executable='cart_pole_controller_am',
        parameters=[
            config_node,
        ]
    )

    return LaunchDescription([
        node
    ])