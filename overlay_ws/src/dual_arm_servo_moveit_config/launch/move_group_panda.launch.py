import os
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("dual_arm_servo")
        .robot_description(file_path="config/panda/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda/panda.srdf")
        .robot_description_kinematics(file_path="config/panda/kinematics.yaml")
        .joint_limits(file_path="config/panda/joint_limits.yaml")
        .trajectory_execution(file_path="config/panda/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_config = os.path.join(
        get_package_share_directory("dual_arm_servo_moveit_config"),
        "launch/moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )
    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("dual_arm_servo_moveit_config"),
        "config/panda/",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "joint_state_broadcaster",
        "left_arm_controller",
        "right_arm_controller",
        "left_hand_controller",
        "right_hand_controller",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # Get parameters for the Servo node
    left_servo_params = {
        "moveit_servo": ParameterBuilder("dual_arm_servo_moveit_config")
        .yaml("config/panda/left_arm_servo_config.yaml")
        .to_dict()
    }
    
    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    left_planning_group_name = {"planning_group_name": "left_arm"}

    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    left_servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        namespace="/left",
        parameters=[
            left_servo_params,
            acceleration_filter_update_period,
            left_planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )
    
    # Get parameters for the Servo node
    right_servo_params = {
        "moveit_servo": ParameterBuilder("dual_arm_servo_moveit_config")
        .yaml("config/panda/right_arm_servo_config.yaml")
        .to_dict()
    }
    
    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    right_planning_group_name = {"planning_group_name": "right_arm"}

    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    right_servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        namespace="/right",
        parameters=[
            right_servo_params,
            acceleration_filter_update_period,
            right_planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )
    
    return LaunchDescription(
        [
            rviz_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            left_servo_node,
            right_servo_node,
        ]
        + load_controllers
    )
