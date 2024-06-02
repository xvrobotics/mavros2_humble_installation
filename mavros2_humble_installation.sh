#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Update package lists
sudo apt update

# Install necessary dependencies
sudo apt install -y python3-vcstool python3-rosinstall-generator python3-osrf-pycommon python3-colcon-common-extensions python3-rosdep git wget

# Source ROS 2 Humble setup script
source /opt/ros/humble/setup.bash

# Create the workspace
mkdir -p ~/mavros2_ws/src
cd ~/mavros2_ws

# Install dependencies
sudo rosdep init || echo "rosdep already initialized"
rosdep update
rosdep install --from-paths src --ignore-src -y

# Install GeographicLib datasets
./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

# Build the workspace
colcon build

# Source the workspace
echo "source ~/mavros2_ws/install/setup.bash" >> ~/.bashrc
source ~/ros2_ws/install/setup.bash

echo "MAVROS installation for ROS 2 Humble completed successfully."

