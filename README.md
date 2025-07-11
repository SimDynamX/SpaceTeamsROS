# SpaceTeams ROS 2 Integration

This repository provides ROS 2 integration for SpaceTeams, a UE5-based space simulation software. It contains service definitions and example code for students participating in the SpaceTeams university competition.

## Overview

SpaceTeams communicates with ROS 2 through a Python script using `roslibpy`, which interfaces with the ROS 2 ecosystem via `rosbridge_server`. This repository provides:

- Custom ROS 2 service definitions for SpaceTeams communication
- Example Python client demonstrating service usage
- Setup scripts for easy installation and configuration
- Bridge setup for UE5 ↔ ROS 2 communication

## Prerequisites

- **Operating System**: Linux (Ubuntu recommended)
- **ROS 2**: Any distribution with rosbridge_server support (Humble, Iron, Jazzy, Rolling)
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

### 4. Test the Setup (Optional)

In another terminal, test the example client:

```bash
# Source your workspace
source install/setup.bash

# Run the example client
ros2 run space_teams_python example_client "Hello SpaceTeams!"
```

## Repository Structure

```
SpaceTeamsROS/
├── first_time_setup.bash          # Initial setup script
├── run_rosbridge.bash             # ROS bridge launcher
├── space_teams_definitions/       # Custom service definitions
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── srv/
│       └── StringService.srv      # String-based service definition
└── space_teams_python/           # Python package with examples
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    └── space_teams_python/
        ├── __init__.py
        └── example_client.py       # Example service client
```

## Service Definitions

### StringService

Located in `space_teams_definitions/srv/StringService.srv`

**Request:**
```
string data
```

**Response:**
```
bool success
```

This service accepts a string message and returns a success status. Use this as a template for creating your own SpaceTeams communication services.

## Usage in SpaceTeams

1. **Start ROS Bridge**: Always run `./run_rosbridge.bash` before launching SpaceTeams
2. **SpaceTeams Connection**: SpaceTeams will connect to the rosbridge WebSocket server (usually `ws://localhost:9090`)
3. **Service Communication**: SpaceTeams can call ROS 2 services and receive responses through the bridge

## Development

### Adding New Services

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
   ```

### Creating Service Servers

Create Python nodes that implement your services. See `example_client.py` for client implementation patterns.

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