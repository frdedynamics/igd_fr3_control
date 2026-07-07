"""
servo.launch.py

Starts moveit_servo's servo_node for continuous FR3 jogging.
Assumes the rest of your FR3 bringup (robot_state_publisher, ros2_control,
move_group, etc.) is already running -- this launches ONLY servo_node.

Adjust package_name / file paths to match your actual franka_fr3_moveit_config.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # URDF/SRDF live in franka_description, not franka_fr3_moveit_config.
    franka_description_path = get_package_share_directory("franka_description")
    urdf_xacro_path = os.path.join(franka_description_path, "robots", "fr3", "fr3.urdf.xacro")
    srdf_xacro_path = os.path.join(franka_description_path, "robots", "fr3", "fr3.srdf.xacro")

    # Matches the arguments used by your working move_group bringup.
    xacro_mappings = {
        "robot_type": "fr3",
        "hand": "true",
        "use_fake_hardware": "false",
        "robot_ip": "192.170.10.101",
    }

    joint_limits_path = os.path.join(franka_description_path, "robots", "fr3", "joint_limits.yaml")
    kinematics_path = os.path.join(franka_description_path, "robots", "fr3", "kinematics.yaml")

    moveit_config = (
        MoveItConfigsBuilder("fr3", package_name="franka_fr3_moveit_config")
        .robot_description(file_path=urdf_xacro_path, mappings=xacro_mappings)
        .robot_description_semantic(file_path=srdf_xacro_path, mappings=xacro_mappings)
        .joint_limits(file_path=joint_limits_path)
        .to_moveit_configs()
    )

    servo_yaml_path = os.path.join(
        get_package_share_directory("igd_fr3_control"),
        "config",
        "servo_config.yaml",
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            servo_yaml_path,
        ],
    )

    return LaunchDescription([servo_node])