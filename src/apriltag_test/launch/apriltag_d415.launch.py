from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('apriltag_test'),
        'config',
        'tags_36h11.yaml'
    )

    return LaunchDescription([

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            output='screen',

            remappings=[
                ('image_rect', '/camera/camera/color/image_raw'),
                ('camera_info', '/camera/camera/color/camera_info'),
            ],

            parameters=[config_file]
        )

    ])
