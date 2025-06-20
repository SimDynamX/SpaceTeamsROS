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

source install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml