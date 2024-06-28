#!/bin/bash
set -e

# Source the ROS setup script
source /opt/ros/iron/setup.bash

# sudo apt-get update
# rosdep update
# rosdep install --from-paths /dev_ws/src --ignore-src -r -y
# sudo rm -rf /var/lib/apt/lists/*

echo "Provided command: $@"

# Execute the provided command
exec $@