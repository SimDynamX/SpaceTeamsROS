| [Rules & Info](https://github.com/SimDynamX/SpaceTeamsPro/discussions/106) | [SpaceTeamsROS (Here)](https://github.com/SimDynamX/SpaceTeamsROS) | [Q&A Discussions Category](https://github.com/SimDynamX/SpaceTeamsPRO/discussions/categories/2025-mars-autonomous-rover-rally-q-a)
| ----- | ----- | ----- |

# Space Teams PRO ROS 2 Integration

This repository provides ROS 2 integration for Space Teams PRO as well as an example navigation algorithm for the Competition.

## Overview

Space Teams PRO communicates with ROS 2 through a Python script using `roslibpy`, which interfaces with the ROS 2 ecosystem via `rosbridge_server`. This repository provides:

- Custom ROS 2 service definitions for Space Teams PRO communication
- Example Python client demonstrating service usage
- Setup scripts for easy installation and configuration
- Bridge setup for ST PRO ↔ ROS 2 communication

## Prerequisites

We are only officially supporting `Windows 11` with `Ubuntu 22.04` / `ROS2 Humble` running in `WSL2` for this competition.

There may be other configurations that also work, possibly with a different linux distro, a different ROS2 version, running linux in a different VM host, or possibly rearranging what computers/VMs are running Windows vs Linux and changing some IPs in the connection code. We will attempt to assist you if you are running in a non-standard configuration but we can't guarantee it will work.

- **Operating System**: Linux (Ubuntu 22.04 in WSL recommended)
    - If using ROS2 Humble on Ubuntu, Ubuntu 22.04 is required (it can be installed in WSL in addition to 24.04 if you already have that)
- **ROS 2**: 
    - Any distribution with rosbridge_server support may work (`Humble`, `Iron`, `Jazzy`, `Rolling`)
    - To install ROS2 Humble, follow this tutorial: https://docs.ros.org/en/humble/Installation.html
    - Download the debian packages; Do not build ROS from source.
    - Follow all of the steps in the tutorial and make sure you install ROS Dev tools as well.
- **Python**: Python 3.9+

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
- Install `cv_bridge` (ROS package) if not already installed
- Install `opencv-python` (Python package) if not already installed
- Initialize and set up `rosdep` to install a few more packages (requires `sudo`)
- Build the `space_teams_definitions` package using colcon

### 3. Start the ROS Bridge

In a new terminal, start the rosbridge server using the launch file:

```bash
source install/setup.bash
ros2 launch space_teams_python rosbridge_image_client.launch.py
```

This will:
- Launch the rosbridge WebSocket server (default port: 9090)
- Start the image client for receiving images from Space Teams PRO (`/CameraRGB` and `/CameraDepth` topics)

**Keep this terminal running** - Space Teams PRO needs the bridge to be active for communication.

### 4. Run the example client

- Make sure that rosbridge is still running
- Start the Competition sim in Space Teams PRO.
- After choosing a role in STP, open up another WSL terminal to run the example client:

The example client demonstrates how to use various rover control services like steering, acceleration, braking, and taking core samples.

Ensure you're in the `SpaceTeamsROS` directory.

Source your workspace
```bash
source install/setup.bash
```

Build the python package
```bash
colcon build --packages-select space_teams_python
```

Run the example client
```bash
ros2 run space_teams_python example_client
```

Note: The image receiver is now automatically launched when you start the ROS bridge using the launch file in step 3.

## Repository Structure

```
SpaceTeamsROS/
├── first_time_setup.bash          # Initial setup script
├── net_test/                      # Network testing scripts
├── scripts/                       # Utility scripts
├── space_teams_definitions/       # ROS2 service definitions package
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── srv/                       # Service definitions
│       ├── Float.srv
│       └── String.srv
└── space_teams_python/            # Python package with examples
    ├── launch/                    # Launch files
    │   └── rosbridge_image_client.launch.py  # Combined rosbridge and image client launcher
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    └── space_teams_python/
        ├── __init__.py
        ├── example_client.py      # Demo navigation algorithm
        ├── image_client.py        # RGB/Depth Image republisher
        └── transformations.py     # Transformation utilities
```

## Service Definitions

Space Teams PRO provides several ROS services for controlling the rover and interacting with the simulation. All services use one of two message types defined below.

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

This service accepts a (64-bit) float message and returns a success status. Used for control commands and parameter settings.

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

This service accepts a (ASCII) string message and returns a success status. Used for logging messages.

## Available Services

### Control Services

| Service Name | Service Type | Description |
|--------------|--------------|-------------|
| `/log_message` | `space_teams_definitions/String` | Sends a log message to the Space Teams PRO console |
| `/Steer` | `space_teams_definitions/Float` | Controls rover steering (-1.0 to 1.0, where -1 is left, 1 is right) |
| `/Accelerator` | `space_teams_definitions/Float` | Controls rover acceleration (0.0 to 1.0) |
| `/Reverse` | `space_teams_definitions/Float` | Controls rover reverse movement |
| `/Brake` | `space_teams_definitions/Float` | Controls rover brake (handbrake is activated when value > 0.5) |
| `/CoreSample` | `space_teams_definitions/Float` | Initiates core sample collection at current location |

### Camera Control Services

| Service Name | Service Type | Description |
|--------------|--------------|-------------|
| `/ChangeFOV` | `space_teams_definitions/Float` | Sets camera field of view (40.0 to 80.0 degrees) |
| `/ChangeExposure` | `space_teams_definitions/Float` | Sets camera exposure in Exposure Value (5.0 to 20.0) |

### Topics

| Topic Name | Message Type | Description |
|------------|--------------|-------------|
| `/CoreSamplingComplete` | `geometry_msgs/Point` | Published when core sampling is completed |
| `/CameraRGB` | `sensor_msgs/Image` | RGB camera feed from rover (published by STP) |
| `/CameraDepth` | `sensor_msgs/Image` | Depth camera feed from rover (published by STP) |


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

### Adding Your Custom Nodes to the Launch File

You can add your own nodes to the launch file to have them start automatically with the rosbridge server and image client. This is useful for your custom rover controllers or any other nodes you want to run alongside the core components.

#### 1. Locate the Launch File

The launch file is located at `space_teams_python/launch/rosbridge_image_client.launch.py`.

#### 2. Edit the Launch File

Open the launch file and add your node to the launch description:

```python
# Inside the generate_launch_description() function
# Add your custom node
your_custom_node = Node(
    package='space_teams_python',  # Or your custom package name
    executable='your_node_executable',  # The name of your Python script (without .py)
    name='your_node_name',
    output='screen',
    # Optional: Add parameters if needed
    parameters=[
        {'param_name': 'param_value'}
    ]
)

# Add your node to the LaunchDescription return statement
return LaunchDescription([
    port_arg,
    rosbridge_launch,
    image_client_node,
    your_custom_node  # Add your node here
])
```

#### 3. Build Your Package

After adding your node to the launch file, rebuild your package:

```bash
colcon build --packages-select space_teams_python
source install/setup.bash
```

#### 4. Launch with Your Custom Node

Run the launch file as normal, and your custom node will start alongside the rosbridge server and image client:

```bash
ros2 launch space_teams_python rosbridge_image_client.launch.py
```

### Adding New Nodes to setup.py

When you create a new Python node for your ROS 2 package, you need to register it in `setup.py` to make it available via the `ros2 run` command. This is crucial for making your nodes discoverable and executable within the ROS 2 ecosystem.

#### 1. Create Your Node File

Create your Python node file in the `space_teams_python/space_teams_python` directory with a `main()` function:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class YourCustomNode(Node):
    def __init__(self):
        super().__init__('your_custom_node')
        # Your node implementation here

def main(args=None):
    rclpy.init(args=args)
    node = YourCustomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 2. Edit setup.py

Open `space_teams_python/setup.py` and add your node to the `entry_points` section:

```python
entry_points={
    'console_scripts': [
        'example_client = space_teams_python.example_client:main',
        'image_client = space_teams_python.image_client:main',
        'your_node_name = space_teams_python.your_node_file:main',  # Add this line
    ],
},
```

Where:
- `your_node_name` is the command name you'll use with `ros2 run`
- `space_teams_python.your_node_file` is the path to your Python module
- `main` is the function that will be executed

#### 3. Rebuild Your Package

**Important:** You must rebuild your package after ANY changes to Python files, launch files, or setup.py:

```bash
colcon build --packages-select space_teams_python
source install/setup.bash
```

This step is required whenever you:
- Create or modify any Python node
- Update launch files
- Change setup.py or package.xml
- Add new dependencies
- Change any resource files

#### 4. Run Your Custom Node

Once built and sourced, you can run your node with:

```bash
ros2 run space_teams_python your_node_name
```

### Creating your own client

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

In your node's `__init__` method, create clients for the Space Teams PRO services:

```python
class YourRoverController(Node):
    def __init__(self):
        super().__init__('your_rover_controller')
        
        # Create service clients for control
        self.logger_client = self.create_client(String, 'log_message')
        self.steer_client = self.create_client(Float, 'Steer')
        self.accelerator_client = self.create_client(Float, 'Accelerator')
        self.reverse_client = self.create_client(Float, 'Reverse')
        self.brake_client = self.create_client(Float, 'Brake')
        self.core_sample_client = self.create_client(Float, 'CoreSample')
        
        # Create service clients for camera control
        self.change_fov_client = self.create_client(Float, 'ChangeFOV')
        self.change_exposure_client = self.create_client(Float, 'ChangeExposure')
        
        # Create subscribers for topics
        self.core_sampling_complete_sub = self.create_subscription(
            Point, 
            'CoreSamplingComplete',
            self.core_sampling_complete_callback,
            10)
        
        # Wait for services to be available
        for client, name in [
            (self.logger_client, 'log_message'),
            (self.steer_client, 'Steer'),
            (self.accelerator_client, 'Accelerator'),
            (self.reverse_client, 'Reverse'),
            (self.brake_client, 'Brake'),
            (self.core_sample_client, 'CoreSample'),
            (self.change_fov_client, 'ChangeFOV'),
            (self.change_exposure_client, 'ChangeExposure')
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {name} not available, waiting...')
                
    def core_sampling_complete_callback(self, msg):
        self.get_logger().info('Core sampling completed!')
```

#### 3. Implement service call methods

Create helper methods to interact with each service:

```python
def log_message(self, message):
    request = String.Request()
    request.data = message
    future = self.logger_client.call_async(request)
    return future

def send_steer_command(self, steer_value):
    request = Float.Request()
    request.data = max(-1.0, min(1.0, steer_value))  # Clamp between -1 and 1
    future = self.steer_client.call_async(request)
    return future

def send_accelerator_command(self, accel_value):
    request = Float.Request()
    request.data = max(0.0, min(1.0, accel_value))  # Clamp between 0 and 1
    future = self.accelerator_client.call_async(request)
    return future

def send_reverse_command(self, reverse_value):
    request = Float.Request()
    request.data = max(0.0, min(1.0, reverse_value))  # Clamp between 0 and 1
    future = self.reverse_client.call_async(request)
    return future

def send_brake_command(self, brake_value):
    request = Float.Request()
    request.data = max(0.0, min(1.0, brake_value))  # Clamp between 0 and 1, > 0.5 activates handbrake
    future = self.brake_client.call_async(request)
    return future

def take_core_sample(self):
    request = Float.Request()
    request.data = 1.0  # The value doesn't matter for this service
    future = self.core_sample_client.call_async(request)
    return future

def set_camera_fov(self, fov):
    request = Float.Request()
    request.data = max(40.0, min(80.0, fov))  # Clamp between 40 and 80 degrees
    future = self.change_fov_client.call_async(request)
    return future

def set_camera_exposure(self, exposure):
    request = Float.Request()
    request.data = max(5.0, min(20.0, exposure))  # Clamp between 5 and 20
    future = self.change_exposure_client.call_async(request)
    return future
```

#### 4. Implement your navigation logic

Use the service methods to create your navigation algorithm. The example client demonstrates a simple coordinate-based navigation system that:
- Gets the current position and orientation
- Calculates bearing and distance to target
- Applies proportional steering control
- Adjusts speed based on proximity and heading error

## Available Services

Space Teams PRO provides the following ROS 2 services for rover control and telemetry:

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
| `log_message`  | String | Sends log messages to Space Teams PRO | Use for debugging and status reporting |

### Service Response Format

All services return a boolean `success` field in addition to their specific data:
- `true`: Service call was successful
- `false`: Service call failed or encountered an error

Example usage patterns are demonstrated in the `example_client.py` file, which implements a complete navigation algorithm using these services.

## Camera Features

Space Teams PRO provides both RGB and depth camera feeds via ROS 2 image topics, allowing you to access and process real-time images and depth data from the rover's onboard cameras. This is useful for computer vision, 3D mapping, navigation, and debugging.

### How to Use the RGB Camera Feed

1. **Subscribe to the Image Topic**
   - The camera publishes RGB images on the topic `/CameraRGB` using the standard `sensor_msgs/Image` message type
   - You can subscribe to this topic in your ROS 2 node:

   ```python
    from sensor_msgs.msg import Image
    self.subscription = self.create_subscription(
        Image,
        '/CameraRGB',
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
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Process the image
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
   ```

3. **Display or Analyze Images**
   - You can display the camera feed using OpenCV:

   ```python
   import cv2
   cv2.imshow('Camera Feed', cv_image)
   cv2.waitKey(1)
   ```
   - You can also analyze pixel values, detect objects, or perform other computer vision tasks


### Depth Camera

Space Teams PRO also provides a depth camera feed that allows you to access distance information for each pixel in the image. This is particularly useful for obstacle detection, terrain analysis, and 3D environment mapping.

1. **Subscribe to the Depth Topic**
   - The depth camera publishes data on the topic `/CameraDepth` using the standard `sensor_msgs/Image` message type
   - Values in the depth image represent the distance in meters from the camera to objects in the scene
   
   ```python
   from sensor_msgs.msg import Image
   self.depth_subscription = self.create_subscription(
       Image,
       '/CameraDepth',
       self.depth_callback,
       10
   )
   ```

2. **Process Depth Data**
   - Use the `cv_bridge` library to convert depth data to a format you can work with:

   ```python
   def depth_callback(self, msg):
       try:
           # Convert to numpy array (depth map)
           depth_image = self.bridge.imgmsg_to_cv2(msg)
           
           # You can access distance values directly from the image
           # For example, to get the distance at the center:
           height, width = depth_image.shape
           center_distance = depth_image[height//2, width//2]
           self.get_logger().info(f'Center distance: {center_distance} meters')
           
           # Visualize the depth map
           # Note: Need to normalize for visualization
           depth_colormap = cv2.applyColorMap(
               cv2.convertScaleAbs(depth_image, alpha=0.03), 
               cv2.COLORMAP_JET
           )
           cv2.imshow('Depth Camera', depth_colormap)
           cv2.waitKey(1)
       except Exception as e:
           self.get_logger().error(f'Error processing depth image: {str(e)}')
   ```

3. **Combined RGB and Depth Processing**
   - You can correlate pixels between RGB and depth images to get rich information about the environment
   - This enables applications like semantic segmentation with distance awareness or object detection with distance estimation

### Notes
- Both camera feeds are real-time and can be used for navigation, obstacle detection, or logging
- The depth data provides crucial information for autonomous navigation and terrain analysis
- You can extend the image client to save images, run ML models, or visualize results

## Troubleshooting

### ROS 2 Not Found

If you encounter issues with ROS 2 not being found:

1. **Verify Installation**: Ensure ROS 2 is properly installed according to the [official documentation](https://docs.ros.org/en/humble/Installation.html)
2. **Check Setup File**: Confirm that `/opt/ros/[DISTRO]/setup.bash` exists on your system
3. **Manually Source ROS 2**: Run this command in your terminal (replace 'humble' with your distribution):
   ```bash
   source /opt/ros/humble/setup.bash
   ```

### Rosbridge Connection Issues

If you're having problems connecting to the rosbridge server:

1. **Verify rosbridge is running**:
   ```bash
   ros2 node list | grep rosbridge
   ```
   You should see `/rosbridge_websocket` in the output

2. **Check WebSocket port is active**:
   ```bash
   netstat -an | grep 9090
   ```
   You should see an entry showing port 9090 is listening

3. **Check firewall settings**: Ensure your firewall allows connections on port 9090

### Build Errors

When encountering build issues:

1. **Missing colcon**:
   
   If you see: `./first_time_setup.bash: line 57: colcon: command not found`
   
   Install the required build dependencies:
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

2. **Clean rebuild**:
   
   Try clearing your build artifacts and rebuilding from scratch:
   ```bash
   rm -rf build install log && colcon build
   ```

### Permission Denied on Scripts

If you encounter permission errors when trying to run scripts:

Make the scripts executable with:
```bash
chmod +x first_time_setup.bash
```

## Supported ROS 2 Distributions

This package supports ROS 2 distributions that include `rosbridge_server`:
- **ROS 2 Humble** (LTS) - Recommended
- **ROS 2 Iron**
- **ROS 2 Jazzy** (LTS)
- **ROS 2 Rolling** (Latest)

## Competition Notes

This repository serves as a starting template for Space Teams University competition participants. The service definitions and examples will be expanded and modified throughout the competition development process.

## License

Apache License 2.0

## Support

For questions specific to Space Teams PRO, refer to the competition documentation. For ROS 2 issues, consult the [ROS 2 documentation](https://docs.ros.org/en/humble/).
