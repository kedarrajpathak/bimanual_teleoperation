import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    
    launch_arguments = {
        "robot_ip": "yyy.yyy.yyy.yyy",
        "use_fake_hardware": "true",
        "sim_isaac": "true",
        "gripper": "robotiq_2f_85",
        "gripper_joint_name": "robotiq_85_left_knuckle_joint",
        "dof": "7",
    }
    moveit_config = (
        MoveItConfigsBuilder("dual_arm_servo")
        .robot_description(file_path="config/kinova/gen3_dual.xacro", mappings=launch_arguments)
        .robot_description_semantic(file_path="config/kinova/gen3_dual.srdf")
        .robot_description_kinematics(file_path="config/kinova/kinematics.yaml")
        .joint_limits(file_path="config/kinova/joint_limits.yaml")
        .trajectory_execution(file_path="config/kinova/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Launch Servo as a standalone node or as a "node component" for better latency/efficiency
    launch_as_standalone_node = LaunchConfiguration(
        "launch_as_standalone_node", default="true"
    )

    # Get parameters for the Servo node
    arm_servo_params = {
        "moveit_servo": ParameterBuilder("dual_arm_servo_moveit_config")
        .yaml("config/kinova/right_arm_servo_config.yaml")
        .to_dict()
    }
    
    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    arm_planning_group_name = {"planning_group_name": "right_arm"}

    # Launch as much as possible in components
    container = launch_ros.actions.ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/right",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Launching as a node component makes ROS 2 intraprocess communication more efficient.
            launch_ros.descriptions.ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                namespace="/right",
                parameters=[
                    arm_servo_params,
                    acceleration_filter_update_period,
                    arm_planning_group_name,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
                condition=UnlessCondition(launch_as_standalone_node),
            ),
        ],
        output="screen",
    )
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    arm_servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        namespace="/right",
        parameters=[
            arm_servo_params,
            acceleration_filter_update_period,
            arm_planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
        condition=IfCondition(launch_as_standalone_node),
    )
    
    return launch.LaunchDescription(
        [
            arm_servo_node,
            container,
        ]
    )
