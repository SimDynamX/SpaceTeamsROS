#!/bin/bash

# make sure ros2 is installed otherwise exit with an error
# if ! command -v ros2 &> /dev/null; then
#     echo "ROS 2 is not installed. Please install ROS 2 before running this script."
#     exit 1
# fi

# Try to detect ROS_DISTRO by searching for installed ROS 2 distributions
if [ -z "$ROS_DISTRO" ]; then
    # Look for setup.bash files in /opt/ros and pick the first one found
    if [ -d "/opt/ros" ]; then
        ROS_DISTRO=$(ls /opt/ros | head -n 1)
    fi
fi

if [ -z "$ROS_DISTRO" ]; then
    echo "Could not determine ROS 2 distribution. Please ensure ROS 2 is installed in /opt/ros."
    exit 1
fi

# try to source the ROS 2 setup.bash
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    echo "Could not find /opt/ros/$ROS_DISTRO/setup.bash. Please check your ROS 2 installation."
    exit 1
fi

#install rosbridge if not already installed
if ! ros2 pkg list | grep -q rosbridge_server; then
    echo "Installing rosbridge_server..."
    sudo apt update
    sudo apt install -y ros-$ROS_DISTRO-rosbridge-server
else
    echo "rosbridge_server is already installed."
fi

# Install cv_bridge if not already installed
if ! ros2 pkg list | grep -q cv_bridge; then
    echo "Installing cv_bridge..."
    sudo apt update
    sudo apt install -y ros-$ROS_DISTRO-cv-bridge
else
    echo "cv_bridge is already installed."
fi

# Install opencv-python in the current Python environment if not already installed
if ! python3 -c "import cv2" &> /dev/null; then
    echo "Installing opencv-python via pip..."
    python3 -m pip install opencv-python
else
    echo "opencv-python is already installed."
fi

# Build the service definitions using colcon
colcon build --packages-select space_teams_definitions