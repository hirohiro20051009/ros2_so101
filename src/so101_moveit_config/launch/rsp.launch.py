from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("so101_new_calib", package_name="so101_moveit_config")
        .robot_description(
            mappings={"use_fake_hardware": LaunchConfiguration("use_fake_hardware")}
        )
        .to_moveit_configs()
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))
    ld.add_action(
        DeclareBooleanLaunchArg(
            "use_fake_hardware",
            default_value=True,
        )
    )

    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            respawn=True,
            output="screen",
            parameters=[
                moveit_config.robot_description,
                {"publish_frequency": LaunchConfiguration("publish_frequency")},
            ],
            remappings=[
                # real時: 実機のjoint_statesをrspに渡す
                ("/joint_states", "/follower/joint_states"),
            ],
        )
    )
    return ld
