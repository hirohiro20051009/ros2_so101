#!/bin/bash

# ros2 setup path
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # Fast DDS -> Cyclone DDS
export RCUTILS_COLORIZED_OUTPUT=1 # Enable colorize output (info, warn, error)

# lerobot python path
export LECONDA_SITE_PACKAGES=/home/hirok/miniforge3/envs/ros2_lerobot/lib/python3.10/site-packages

# workspace
source ~/ros2_so101/install/setup.bash

