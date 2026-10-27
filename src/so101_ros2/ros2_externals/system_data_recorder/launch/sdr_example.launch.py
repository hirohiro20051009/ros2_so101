import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the package
    sdr_pkg_path = get_package_share_directory('system_data_recorder') # <-- CHANGE 'your_package_name'

    # Get the path to the YAML file
    config_file_path = os.path.join(sdr_pkg_path, 'config', 'sdr_example.yaml')

    # Create the node action
    sdr_node = Node(
        package='system_data_recorder', # <-- CHANGE 'your_package_name'
        executable='system_data_recorder', # <-- CHANGE to your C++ executable name from CMakeLists.txt
        name='sdr',
        output='screen',
        parameters=[config_file_path]
    )

    return LaunchDescription([
        sdr_node
    ])