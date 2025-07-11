# SpaceTeams ROS2 Packages

This repository contains two ROS2 packages for the SpaceTeams project:

1. **space_teams_definitions** - Service definition package
2. **space_teams_python** - Python implementation package with server and client

## Package Structure

```
all_in_one/
├── space_teams_definitions/        # Service definitions package
│   ├── srv/
│   │   └── MoveLeft.srv           # MoveLeft service definition
│   ├── CMakeLists.txt
│   └── package.xml
└── space_teams_python/             # Python implementation package
    ├── space_teams_python/
    │   ├── __init__.py
    │   ├── move_left_server.py    # Service server
    │   └── move_left_client.py    # Service client
    ├── resource/
    │   └── space_teams_python
    ├── package.xml
    ├── setup.py
    └── setup.cfg
```

## Service Definition

The `MoveLeft` service has the following interface:

**Request:**
- `float64 distance` - Distance to move
- `string direction_type` - Type of direction movement

**Response:**
- `bool success` - Whether the operation was successful
- `string message` - Status message
- `float64 actual_distance` - Actual distance moved

## Building the Packages

1. Make sure you have ROS2 installed and sourced
2. Navigate to your ROS2 workspace
3. Copy these packages to your `src` directory
4. Build the packages:

```bash
# Build the service definition package first
colcon build --packages-select space_teams_definitions

# Source the workspace to make the service definitions available
source install/setup.bash

# Build the Python package
colcon build --packages-select space_teams_python

# Source again to make the Python package available
source install/setup.bash
```

## Running the Service

### Terminal 1 - Start the Server:
```bash
source install/setup.bash
ros2 run space_teams_python move_left_server
```

### Terminal 2 - Run the Client:
```bash
source install/setup.bash
# Basic usage (default: distance=5.0, direction_type='standard')
ros2 run space_teams_python move_left_client

# With custom parameters
ros2 run space_teams_python move_left_client 10.5 precise
```

### Terminal 3 - Test with ros2 service (optional):
```bash
source install/setup.bash
# List available services
ros2 service list

# Call the service directly
ros2 service call /move_left space_teams_definitions/srv/MoveLeft "{distance: 3.0, direction_type: 'manual'}"
```

## Features

- **Service Server**: Handles MoveLeft requests with validation and logging
- **Service Client**: Sends requests with configurable parameters
- **Error Handling**: Validates distance values and provides meaningful error messages
- **Logging**: Comprehensive logging for debugging and monitoring
- **Configurable**: Client accepts command-line arguments for distance and direction type

## Dependencies

- ROS2 (tested with Humble/Iron)
- Python 3
- rclpy
- Custom service definitions from space_teams_definitions package

## Troubleshooting

### Build Issues
- If you get errors about missing section headers in setup.cfg, make sure the setup.cfg file contains proper configuration format, not Python code
- Make sure to build the service definition package first before building the Python package
- Always source the workspace after building the definitions package

### Runtime Issues
- If the service is not found, make sure both packages are built and sourced
- Check that the service server is running before trying to call it with the client
