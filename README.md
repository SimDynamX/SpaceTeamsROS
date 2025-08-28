# SpaceTeams ROS 2 Integration

This repository provides ROS 2 integration for SpaceTeams as well as an example navigation algorithm for the Competition.

## Overview

SpaceTeams communicates with ROS 2 through a Python script using `roslibpy`, which interfaces with the ROS 2 ecosystem via `rosbridge_server`. This repository provides:

- Custom ROS 2 service definitions for SpaceTeams communication
- Example Python client demonstrating service usage
- Setup scripts for easy installation and configuration
- Bridge setup for UE5 ↔ ROS 2 communication

## Prerequisites

- **Operating System**: Linux (Ubuntu in WSL recommended)
- **ROS 2**: Any distribution with rosbridge_server support (Humble, Iron, Jazzy, Rolling)
    - To install ROS2 Humble, follow this tutorial: https://docs.ros.org/en/humble/Installation.html
- **Python**: Python 3.6+
- **Internet connection** for package installation

## Quick Start

### 1. Clone the Repository

```bash
git clone https://github.com/SimDynamX/SpaceTeamsROS.git
cd SpaceTeamsROS
```

### 2. First Time Setup

Run the setup script to install dependencies and build the project:

```bash
chmod +x first_time_setup.bash
./first_time_setup.bash
```

This script will:
- Auto-detect your ROS 2 distribution
- Source the ROS 2 environment
- Install `rosbridge_server` if not already installed
- Build the `space_teams_definitions` package using colcon

### 3. Start the ROS Bridge

In a new terminal, start the rosbridge server:

```bash
chmod +x run_rosbridge.bash
./run_rosbridge.bash
```

This will:
- Source the ROS 2 environment
- Source the local workspace
- Launch the rosbridge WebSocket server (default port: 9090)

**Keep this terminal running** - SpaceTeams needs the bridge to be active for communication.

### 4. Run the demo (Optional)

Make sure that rosbridge is still running and start the Competition sim.
After choosing a pawn, open up linux another terminal run the example client:

```bash
# Source your workspace
cd SpaceTeamsROS
source install/setup.bash

# build the python package
colcon build --packages-select space_teams_python

# Run the example client
ros2 run space_teams_python example_client
```

## Repository Structure

```
SpaceTeamsROS/
├── first_time_setup.bash          # Initial setup script
├── run_rosbridge.bash             # ROS bridge launcher
├── space_teams_definitions/       # Custom service definitions
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── srv/                       # Service defenitions
        ├── Float.srv
    ├── String.srv
    # (Custom service definitions for other features, if needed)
└── space_teams_python/            # Python package with examples
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    └── space_teams_python/
        ├── __init__.py
        ├── example_client.py       # Demo navigation algorithm
        └── image_client.py         # Demo camera usage
```

## Service Definitions

### Float

Located in `space_teams_definitions/srv/Float.srv`

**Request:**
```
Float data
```

**Response:**
```
bool success
```

This service accepts a float message and returns a success status. This is used for the steering, acceleration, and brake services.

### String

Located in `space_teams_definitions/srv/String.srv`

**Request:**
```
string data
```

**Response:**
```
bool success
```

This service accepts a string message and returns a success status. This is used for the loggerInfo service.


## Development

<!-- ### Adding New Services

1. Create a new `.srv` file in `space_teams_definitions/srv/`
2. Add the service to `CMakeLists.txt`:
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     "srv/StringService.srv"
     "srv/YourNewService.srv"  # Add this line
     DEPENDENCIES
   )
   ```
3. Rebuild the package:
   ```bash
   colcon build --packages-select space_teams_definitions
   source install/setup.bash
   ``` -->

### Creating your own client

def get_current_location(self):
def send_steer_command(self, steer_value):
def send_accelerator_command(self, accel_value):
The `example_client.py` provides a good blueprint of what a client node interacting with SpaceTeams should look like. To create your own ROS node that interacts with SpaceTeams, follow these steps:

#### 1. Set up your Python ROS 2 node

Create a new Python file in your package and import the necessary modules:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from space_teams_definitions.srv import String, Float
from geometry_msgs.msg import Point, Quaternion
```

#### 2. Create service clients

In your node's `__init__` method, create clients for the SpaceTeams services:

```python
class YourRoverController(Node):
    def __init__(self):
        super().__init__('your_rover_controller')
        # Create service clients
        self.logger_client = self.create_client(String, 'log_message')
        self.steer_client = self.create_client(Float, 'Steer')
        self.accelerator_client = self.create_client(Float, 'Accelerator')
        self.brake_client = self.create_client(Float, 'Brake')
    # self.location_client = self.create_client(Vector3d, 'GetLocation')  # (Removed, now using topics)
    # self.rotation_client = self.create_client(Quaternion, 'GetRotation')  # (Removed, now using topics)
        # Wait for services to be available
        for client, name in [
            (self.logger_client, 'log_message'),
            (self.steer_client, 'Steer'),
            (self.accelerator_client, 'Accelerator'),
            (self.brake_client, 'Brake'),
            (self.location_client, 'GetLocation'),
            (self.rotation_client, 'GetRotation')
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {name} not available, waiting...')
```

#### 3. Implement service call methods

Create helper methods to interact with each service:

```python
def log_message(self, message):
    request = String.Request()
    request.data = message
    future = self.logger_client.call_async(request)
    return future

def get_current_rotation(self):
## (Removed get_current_location and get_current_rotation service calls; now use topic callbacks)

def send_steer_command(self, steer_value):
    request = Float.Request()
    request.data = max(-1.0, min(1.0, steer_value))
    future = self.steer_client.call_async(request)
    return future

def send_accelerator_command(self, accel_value):
    request = Float.Request()
    request.data = max(0.0, min(1.0, accel_value))
    future = self.accelerator_client.call_async(request)
    return future

def send_brake_command(self, brake_value):
    request = Float.Request()
    request.data = max(0.0, min(1.0, brake_value))
    future = self.brake_client.call_async(request)
    return future
```

#### 4. Implement your navigation logic

Use the service methods to create your navigation algorithm. The example client demonstrates a simple coordinate-based navigation system that:
- Gets the current position and orientation
- Calculates bearing and distance to target
- Applies proportional steering control
- Adjusts speed based on proximity and heading error

## Available Services

SpaceTeams provides the following ROS 2 services for rover control and telemetry:

### Control Services

| Service Name   | Type  | Description                  | Input Range                                 |
|---------------|-------|------------------------------|---------------------------------------------|
| `Steer`       | Float | Controls rover steering      | -1.0 to 1.0 (negative = left, positive = right) |
| `Accelerator` | Float | Controls rover acceleration  | 0.0 to 1.0 (0 = no acceleration, 1 = full throttle) |
| `Brake`       | Float | Controls rover braking       | 0.0 to 1.0 (0 = no brake, 1 = full brake)   |

### Telemetry Services

| Service Name   | Type     | Description                   | Response Fields                |
|---------------|----------|-------------------------------|-------------------------------|
| `Location`  | geometry_msgs/Point | Rover's current position (topic) | `x`, `y`, `z` coordinates      |
| `Rotation`  | geometry_msgs/Quaternion | Rover's current orientation (topic) | `x`, `y`, `z`, `w` quaternion components |

### Logging Service

| Service Name   | Type   | Description                  | Usage                        |
|---------------|--------|------------------------------|------------------------------|
| `log_message`  | String | Sends log messages to SpaceTeams | Use for debugging and status reporting |

### Service Response Format

All services return a boolean `success` field in addition to their specific data:
- `true`: Service call was successful
- `false`: Service call failed or encountered an error

Example usage patterns are demonstrated in the `example_client.py` file, which implements a complete navigation algorithm using these services.

## Camera Features

SpaceTeams provides a camera feed via ROS 2 image topics, allowing you to access and process real-time images from the rover's onboard camera. This is useful for computer vision, navigation, and debugging.

### How to Use the Camera Feed

1. **Subscribe to the Image Topic**
   - The camera publishes images on the topic `camera/image_raw` using the standard `sensor_msgs/msg/Image` message type.
   - You can subscribe to this topic in your ROS 2 node:

   ```python
   from sensor_msgs.msg import Image
   self.subscription = self.create_subscription(
       Image,
       'camera/image_raw',
       self.image_callback,
       10
   )
   ```

2. **Process Incoming Images**
   - Use the `cv_bridge` library to convert ROS Image messages to OpenCV format for easy processing:

   ```python
   from cv_bridge import CvBridge
   self.bridge = CvBridge()
   def image_callback(self, msg):
       cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
       # Now you can use OpenCV functions on cv_image
   ```

3. **Display or Analyze Images**
   - You can display the camera feed using OpenCV:

   ```python
   import cv2
   cv2.imshow('Camera Feed', cv_image)
   cv2.waitKey(1)
   ```
   - You can also analyze pixel values, detect objects, or perform other computer vision tasks


### Example: Minimal Image Client

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageClient(Node):
    def __init__(self):
        super().__init__('image_client')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('Camera Feed', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_client = ImageClient()
    rclpy.spin(image_client)
    image_client.destroy_node()
    rclpy.shutdown()
```

### Notes
- The camera feed is real-time and can be used for navigation, obstacle detection, or logging.
- You can extend the image client to save images, run ML models, or visualize results.
- Make sure to install `cv_bridge` and `opencv-python` in your ROS environment.

## Troubleshooting

### ROS 2 Not Found
- Ensure ROS 2 is properly installed
- Check that `/opt/ros/[DISTRO]/setup.bash` exists
- Manually source ROS 2: `source /opt/ros/humble/setup.bash` (replace 'humble' with your distribution)

### Rosbridge Connection Issues
- Verify rosbridge is running: `ros2 node list | grep rosbridge`
- Check WebSocket port: `netstat -an | grep 9090`
- Ensure firewall allows port 9090

### Build Errors
- Install build dependencies: `sudo apt install python3-colcon-common-extensions`
- Clean and rebuild: `rm -rf build install log && colcon build`

### Permission Denied on Scripts
```bash
chmod +x first_time_setup.bash
chmod +x run_rosbridge.bash
```

## Supported ROS 2 Distributions

This package supports ROS 2 distributions that include `rosbridge_server`:
- **ROS 2 Humble** (LTS) - Recommended
- **ROS 2 Iron**
- **ROS 2 Jazzy** (LTS)
- **ROS 2 Rolling** (Latest)

## Competition Notes

This repository serves as a starting template for SpaceTeams competition participants. The service definitions and examples will be expanded and modified throughout the competition development process.

## License

Apache License 2.0

## Support

For SpaceTeams-specific questions, refer to the competition documentation. For ROS 2 issues, consult the [ROS 2 documentation](https://docs.ros.org/en/humble/).
