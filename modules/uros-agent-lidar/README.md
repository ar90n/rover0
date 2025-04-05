# Micro-ROS Agent LiDAR Module

This module provides the bridge between the LiDAR microcontroller and the ROS2 environment, enabling LiDAR data to be used for mapping and navigation.

## Overview

The Micro-ROS Agent LiDAR module is responsible for:

- Establishing communication between the LiDAR microcontroller and ROS2
- Translating LiDAR data into ROS2 messages
- Managing the serial connection to the LiDAR hardware
- Providing a standardized interface for LiDAR data consumption

## Architecture

The module follows the Micro-ROS architecture:

1. **Micro-ROS Client**: Runs on the LiDAR microcontroller (Raspberry Pi Pico)
2. **Micro-ROS Agent**: Runs on the main computer (Raspberry Pi)
3. **ROS2 Environment**: Consumes LiDAR data for mapping and navigation

## Components

### Micro-ROS Agent

The Micro-ROS Agent acts as a bridge between the microcontroller and ROS2, handling:

- Serial communication with the LiDAR microcontroller
- Message serialization/deserialization
- QoS management
- Resource management

### LiDAR Integration

The module integrates with the XV11 LiDAR sensor, which:

- Provides 360-degree distance measurements
- Operates at configurable RPM
- Connects via serial interface
- Delivers data in a specialized format that is translated to ROS2 LaserScan messages

## Docker Integration

The module includes a Dockerfile that:

- Sets up the ROS2 environment
- Installs the Micro-ROS Agent
- Configures serial communication
- Provides an entrypoint for launching the agent

## Usage

### Building

```bash
cd modules/uros-agent-lidar
docker build -t rover0-uros-agent-lidar .
```

### Running

```bash
# Basic operation
docker run --device=/dev/ttyUSB0:/dev/ttyUSB0 rover0-uros-agent-lidar serial -D /dev/ttyUSB0 serial -b 230400 -v1

# With custom parameters
docker run --device=/dev/ttyUSB0:/dev/ttyUSB0 -e LIDAR_SERIAL_PORT=/dev/ttyUSB0 rover0-uros-agent-lidar serial -D $LIDAR_SERIAL_PORT serial -b 230400 -v1
```

### Parameters

Key parameters for the Micro-ROS Agent:

- `-D`: Serial device path
- `-b`: Baud rate (default: 230400)
- `-v`: Verbosity level (0-9)

## ROS2 Integration

The module publishes LiDAR data as:

- **Topic**: `/scan`
- **Message Type**: `sensor_msgs/LaserScan`
- **Frame ID**: `rover0_lidar_link`

This data can be consumed by:

- Navigation stack for obstacle avoidance
- SLAM algorithms for mapping
- Visualization tools for debugging

## Development

### Prerequisites

- ROS2 Jazzy
- Micro-ROS Agent
- Serial communication tools

### Building from Source

```bash
# Clone the repository
git clone https://github.com/ar90n/rover0.git

# Build the packages
cd rover0/modules/uros-agent-lidar
colcon build
```

### Testing the Connection

To verify the connection to the LiDAR microcontroller:

```bash
# Run the agent
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/ttyUSB0 -b 230400

# In another terminal, check for LiDAR data
ros2 topic echo /scan
```

## Troubleshooting

### Common Issues

1. **Serial connection errors**: Check device permissions and baud rate settings
2. **No LiDAR data**: Verify the LiDAR microcontroller is powered and running correctly
3. **Intermittent data**: Check for loose connections or power issues
4. **Incorrect measurements**: Calibrate the LiDAR sensor or check for obstructions

### Debugging

- Use `-v9` parameter for verbose logging
- Monitor serial traffic with tools like `minicom` or `screen`
- Check system logs for USB/serial errors

## Integration with Other Modules

The Micro-ROS Agent LiDAR module integrates with:

- **Rover0 Core Module**: Provides LiDAR data for navigation
- **Teleop Module**: Enables visualization of LiDAR data in the web interface

## License

This module is part of the Rover0 project and is licensed under the MIT License. See the LICENSE file in the root directory for details.