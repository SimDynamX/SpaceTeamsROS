# SpaceTeams Rover Control System

This repository contains a rover control system that integrates the SpaceTeams simulation environment with ROS2 for remote operation. The system consists of two main components:

1. **RoverDriving.py** - Main rover simulation controller that connects to SpaceTeams and provides ROS2 services
2. **ROS2 Packages** - Service definitions and client utilities for rover control

## Repository Structure

```
all_in_one/
├── RoverDriving.py                 # Main rover controller (SpaceTeams + ROS2 bridge)
├── first_time_setup.bash           # Initial setup script
├── run_rosbridge.bash              # Script to launch ROS bridge server
├── space_teams_definitions/        # ROS2 service definitions
│   ├── srv/
│   │   ├── MoveLeft.srv            # Rover movement service
│   │   └── StringService.srv       # Logging service
│   ├── CMakeLists.txt
│   └── package.xml
├── space_teams_python/             # ROS2 Python utilities
│   ├── space_teams_python/
│   │   ├── __init__.py
│   │   └── example_client.py       # Example ROS2 client
│   ├── resource/
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
├── build/                          # Colcon build artifacts
├── install/                        # Colcon install artifacts
└── log/                           # Build logs
```

## System Overview

This system creates a bridge between the SpaceTeams lunar rover simulation and ROS2, enabling:

- **Lunar Surface Navigation**: The rover navigates between predefined waypoints on the Moon using elevation data
- **Remote Control**: ROS2 services allow external systems to control rover acceleration
- **Web Interface**: ROSBridge enables web-based control through WebSocket connections
- **Real-time Logging**: Comprehensive logging for debugging and monitoring

## Service Definitions

### MoveLeft Service
**Request:**
- `float64 data` - Accelerator command value

**Response:**
- `bool success` - Whether the operation was successful

### StringService
**Request:**
- `string data` - Message to log

**Response:**
- `bool success` - Whether logging was successful

## Setup and Installation

### Prerequisites
- ROS2 (Humble/Iron recommended)
- Python 3
- SpaceTeams simulation environment
- Linux/WSL environment (for bash scripts)

### First-Time Setup

1. **Run the setup script** (installs ROSBridge and builds service definitions):
```bash
./first_time_setup.bash
```

This script will:
- Detect your ROS2 distribution
- Install `rosbridge_server` if not already installed
- Build the `space_teams_definitions` package

2. **Build the Python package** (optional, for using example clients):
```bash
colcon build --packages-select space_teams_python
source install/setup.bash
```

## Running the System

The system requires three components to be running:

### 1. Start ROSBridge Server
```bash
./run_rosbridge.bash
```
This launches the ROS-to-WebSocket bridge on port 9090.

### 2. Start the SpaceTeams Rover Simulation
```bash
python RoverDriving.py [simulation_arguments]
```
This connects to the SpaceTeams simulation and:
- Loads lunar elevation data
- Sets up waypoint navigation
- Starts the ROS2 service server for `/move_left`
- Provides accelerator control via ROS2 services

### 3. Control the Rover (Optional)

#### Using the Example Client:
```bash
source install/setup.bash
ros2 run space_teams_python example_client "Hello SpaceTeams!"
```

#### Using ROS2 Service Commands:
```bash
# List available services
ros2 service list

# Send accelerator command
ros2 service call /move_left space_teams_definitions/srv/MoveLeft "{data: 0.5}"

# Send log message
ros2 service call /log_message space_teams_definitions/srv/StringService "{data: 'Custom log message'}"
```

#### Using Web Interface (via ROSBridge):
Connect to `ws://localhost:9090` and send JSON commands:
```json
{
  "op": "call_service",
  "service": "/move_left",
  "args": {
    "data": 0.5
  }
}
```

## Key Features

- **Lunar Terrain Navigation**: Uses multi-resolution elevation data for realistic lunar surface navigation
- **Waypoint Following**: Automatically navigates between predefined latitude/longitude waypoints
- **ROS2 Integration**: Provides standard ROS2 services for rover control
- **Web Interface Support**: ROSBridge enables browser-based control and monitoring
- **Real-time Control**: Direct accelerator control with immediate response
- **Comprehensive Logging**: Built-in logging system for debugging and telemetry
- **Fault Tolerance**: Error handling and connection management

## Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Web Client    │    │   ROS2 Client    │    │ SpaceTeams Sim  │
│   (Browser)     │    │   (Python/C++)   │    │   Environment   │
└─────────┬───────┘    └─────────┬────────┘    └─────────┬───────┘
          │                      │                       │
          │ WebSocket            │ ROS2 Services         │ Native API
          │ (Port 9090)          │                       │
          │                      │                       │
      ┌───▼──────────────────────▼───────────────────────▼───┐
      │              RoverDriving.py                         │
      │  ┌─────────────┐  ┌──────────────┐  ┌─────────────┐ │
      │  │ ROSBridge   │  │ ROS2 Services│  │ SpaceTeams  │ │
      │  │ Server      │  │    Server    │  │ Integration │ │
      │  └─────────────┘  └──────────────┘  └─────────────┘ │
      └─────────────────────────────────────────────────────┘
```

## Dependencies

- **ROS2** (Humble/Iron recommended)
- **Python 3.8+**
- **SpaceTeams Simulation Framework**
- **roslibpy** - ROS client library for Python
- **rosbridge_server** - WebSocket interface for ROS
- **numpy** - Numerical computations
- **Custom packages**:
  - `space_teams_definitions` - Service message definitions
  - `space_teams_python` - Python utilities

## Troubleshooting

### Build Issues
- **ROS2 not found**: Ensure ROS2 is properly installed and sourced
- **Service definitions not found**: Run `./first_time_setup.bash` to build the definitions package
- **Colcon build fails**: Make sure you're in the correct workspace directory

### Runtime Issues
- **ROSBridge connection failed**: 
  - Check if `./run_rosbridge.bash` is running
  - Verify port 9090 is not blocked by firewall
  - Ensure ROSBridge server is installed: `sudo apt install ros-$ROS_DISTRO-rosbridge-server`

- **SpaceTeams simulation issues**:
  - Verify SpaceTeams framework is properly installed
  - Check that lunar elevation data files are accessible
  - Ensure simulation arguments are correctly passed to `RoverDriving.py`

- **Service not available**:
  - Confirm `RoverDriving.py` is running and connected to simulation
  - Check ROS2 service list: `ros2 service list`
  - Verify workspace is sourced: `source install/setup.bash`

### Development Tips
- Use `ros2 topic echo` to monitor service calls
- Check logs in the `log/` directory for build issues
- Monitor SpaceTeams logger output for simulation debugging
- Test services individually before integrating with web interfaces

## File Descriptions

- **`RoverDriving.py`**: Main simulation controller, connects SpaceTeams to ROS2
- **`first_time_setup.bash`**: Automated setup script for dependencies and initial build
- **`run_rosbridge.bash`**: Launches ROSBridge WebSocket server
- **`space_teams_definitions/`**: ROS2 message and service definitions
- **`space_teams_python/`**: Example clients and utilities
- **`build/`, `install/`, `log/`**: Colcon workspace artifacts (auto-generated)
