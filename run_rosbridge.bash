#!/bin/bash
# Find and source ROS 2
if [ -z "$ROS_DISTRO" ] && [ -d "/opt/ros" ]; then
    ROS_DISTRO=$(ls /opt/ros | head -n 1)
fi
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash

# Set up cleanup
cleanup() {
    echo "Stopping processes..."
    pkill -f "rosbridge_websocket"
    pkill -f "Image_server.py"
    
        # Use stronger signal if processes remain
    pkill -9 -f "rosbridge_websocket" 2>/dev/null
    pkill -9 -f "Image_server.py" 2>/dev/null
    
    echo "Cleanup complete."
    exit 0
}

trap cleanup SIGINT SIGTERM

# Start rosbridge server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!
echo "Started rosbridge server with PID $ROSBRIDGE_PID"

# Start image server
python3 scripts/Image_server.py &
IMAGE_SERVER_PID=$!
echo "Started image server with PID $IMAGE_SERVER_PID"
echo "All services started. Press Ctrl+C to stop."

# Wait indefinitely until a signal is received
while true; do
    sleep 1
done

