import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('aruco_pose_estimation'),
        'config',
        'config.yaml'
    )

    print("Config file path: ", config)

    return LaunchDescription([
        Node(
            package='aruco_pose_estimation',
            executable='aruco_pose_estimation_node',
            name='aruco_pose_estimation_node',
            parameters=[config]
        )
    ])