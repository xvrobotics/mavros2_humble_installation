#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Update package lists
sudo apt update

sudo apt upgrade

ROS2_WS=~/mavros2_ws

# Delete existing install, log, and build directories if they exist
if [ -d "$ROS2_WS/install" ]; then
  rm -rf $ROS2_WS/install
fi

if [ -d "$ROS2_WS/log" ]; then
  rm -rf $ROS2_WS/log
fi

if [ -d "$ROS2_WS/build" ]; then
  rm -rf $ROS2_WS/build
fi

# Install necessary dependencies
sudo apt install -y python3-vcstool python3-rosinstall-generator python3-osrf-pycommon python3-colcon-common-extensions python3-rosdep git wget

if [ -d "/opt/ros/foxy/" ]; then
  source /opt/ros/foxy/setup.bash
  sudo apt install ros-foxy-mavros
fi

if [ -d "/opt/ros/humble/" ]; then
  source /opt/ros/humble/setup.bash
  sudo apt install ros-humble-mavros
fi
ros2 run mavros install_geographiclib_datasets.sh

# Alternative:
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh

chmod +x ./install_geographiclib_datasets.sh 
sudo ./install_geographiclib_datasets.sh 

# Build the workspace
colcon build

# Source the workspace
echo "source ~/mavros2_ws/install/setup.bash" >> ~/.bashrc
source ~/mavros2_ws/install/setup.bash

echo "MAVROS installation for ROS 2 completed successfully."


