import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "so101_new_calib",
            package_name="so101_moveit_config",
        )
        .to_moveit_configs()
    )

    servo_yaml = load_yaml(
        "so101_moveit_config",
        "config/servo.yaml",
    )

    servo_params = {
        "moveit_servo": servo_yaml,
    }

    return LaunchDescription(
        [
            Node(
                package="moveit_servo",
                executable="servo_node_main",
                name="servo_node",
                output="screen",
                parameters=[
                    servo_params,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ],
            )
        ]
    )

