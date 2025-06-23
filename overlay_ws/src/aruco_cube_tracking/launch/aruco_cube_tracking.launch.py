import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('aruco_cube_tracking'),
        'config',
        'config.yaml'
    )

    print("Config file path: ", config)

    return LaunchDescription([
        Node(
            package='aruco_cube_tracking',
            executable='aruco_cube_tracking_node',
            name='aruco_cube_tracking_node',
            parameters=[config]
        )
    ])