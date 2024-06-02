# mavros2__ws
This repository provides a script to install MAVROS for ROS 2 Foxy and Humble. MAVROS is a ROS package that provides communication between ROS and MAVLink-supported autopilots, such as PX4 and ArduPilot.

## Prerequisites

Ensure you have ROS 2 Foxy installed on your system. If not, follow the installation guide [here](https://docs.ros.org/en/foxy/Installation.html).

Ensure you have ROS 2 Humble installed on your system. If not, follow the installation guide [here](https://docs.ros.org/en/humble/Installation.html).

## Installation

Follow the steps below to set up and install MAVROS for ROS 2 Foxy.

### Step 1: Clone this Repository

Clone this repository to your local machine:

```
git clone https://github.com/your-username/mavros2_foxy_installation.git
cd mavros2_ws
```

### Step 2: Make the Script Executable

Make the installation script executable:
```
chmod +x mavros2_foxy_install.sh
```
```
chmod +x mavros2_humble_install.sh
```
### Step 3: Run the Installation Script

Run the script to install MAVROS:
```
./mavros2_foxy_install.sh
```
```
./mavros2_humble_install.sh
```
This script will perform the following actions:

    Update package lists.
    Install necessary dependencies.
    Set up the ROS 2 workspace.
    Install MAVLink and MAVROS using rosinstall_generator and vcs.
    Install GeographicLib datasets.
    Build the workspace using colcon.

### Step 4: Source the Workspace

After the installation, source the workspace to your current session:
```
source ~/mavros2_ws/install/setup.bash
```
### Verification

To verify the installation, you can run the following command to check if MAVROS nodes are available:
```
ros2 launch mavros px4.launch
```
### Troubleshooting

If you encounter any issues during installation or usage, please check the ROS and MAVROS documentation or open an issue in this repository.
