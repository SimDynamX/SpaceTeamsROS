#!/bin/bash

# make sure ros2 is installed otherwise exit with an error
# if ! command -v ros2 &> /dev/null; then
#     echo "ROS 2 is not installed. Please install ROS 2 before running this script."
#     exit 1
# fi

echo "[ST] Checking ROS distribution"
# Try to detect ROS_DISTRO by searching for installed ROS 2 distributions
if [ -z "$ROS_DISTRO" ]; then
    # Look for setup.bash files in /opt/ros and pick the first one found
    if [ -d "/opt/ros" ]; then
        ROS_DISTRO=$(ls /opt/ros | head -n 1)
    fi
fi

if [ -z "$ROS_DISTRO" ]; then
    echo "[ST] Could not determine ROS 2 distribution. Please ensure ROS 2 is installed in /opt/ros."
    exit 1
else
    echo "[ST] ROS 2 Distribution set properly ($ROS_DISTRO)."
fi

# try to source the ROS 2 setup.bash
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "[ST] Sourcing ros setup at /opt/ros/$ROS_DISTRO/setup.bash ..."
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    echo "[ST] Could not find /opt/ros/$ROS_DISTRO/setup.bash. Please check your ROS 2 installation."
    exit 1
fi

#install rosbridge if not already installed
if ! ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then
    echo "[ST] Installing rosbridge_server..."
    sudo apt update
    sudo apt install -y ros-$ROS_DISTRO-rosbridge-server
else
    echo "[ST] rosbridge_server is already installed."
fi

# Install cv_bridge if not already installed
if ! ros2 pkg prefix cv_bridge >/dev/null 2>&1; then
    echo "[ST] Installing cv_bridge..."
    sudo apt update
    sudo apt install -y ros-$ROS_DISTRO-cv-bridge
else
    echo "[ST] cv_bridge is already installed."
fi

# echo "[ST] Ensuring python pip packages are installed..."

# python3 -m ensurepip
# python3 -m pip install opencv-python zmq numpy

# echo "[ST] Done with python pip packages."

# Install opencv-python in the current Python environment if not already installed
if ! python3 -c "import cv2" &> /dev/null; then
    echo "[ST] Installing opencv-python via pip..."
    python3 -m pip install opencv-python
else
    echo "[ST] opencv-python is already installed."
fi

# # Install zmq in the current Python environment if not already installed
# if ! python3 -c "import zmq" &> /dev/null; then
#     echo "[ST] Installing zmq via pip..."
#     python3 -m pip install zmq
# else
#     echo "[ST] zmq is already installed."
# fi

# rosdep install if not installed
if ! command -v rosdep &> /dev/null; then
    echo "[ST] Installing rosdep..."
    sudo apt update
    sudo apt install -y python3-rosdep
fi

# rosdep setup
sudo rosdep init
rosdep update
rosdep install --from-paths space_teams_python --ignore-src -r -y

# rosdep setup
sudo rosdep init
rosdep update
rosdep install --from-paths space_teams_python --ignore-src -r -y

# Build the service definitions using colcon
echo "[ST] Doing colcon build of space_teams_definitions..."
colcon build --packages-select space_teams_definitions
echo "[ST] Colcon build of space_teams_definitions done."

# Make the run_rosbridge.bash script executable
chmod +x run_rosbridge.bash
