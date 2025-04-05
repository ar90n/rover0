# Teleop Module

This module provides teleoperation capabilities for the Rover0 platform, including camera streaming, joystick control, and web-based visualization.

## Overview

The Teleop module is responsible for:

- Streaming camera feed from the Raspberry Pi Camera
- Processing joystick inputs for remote control
- Providing a web interface for visualization and control
- Integrating with Foxglove Studio for advanced visualization

## Components

### ROS2 Packages

- **teleop_bringup**: Launch files and configurations for the teleoperation system
- **ros_camera**: Interface for the Raspberry Pi Camera
- **foxglove_bridge**: WebSocket server for Foxglove Studio integration

### Third-party Components

- **libcamera**: Raspberry Pi camera stack
- **rpicam-apps**: Camera applications for Raspberry Pi

## Features

### Camera Streaming

- Supports Raspberry Pi Camera Module
- Configurable resolution and frame rate
- Camera calibration for accurate visualization

### Remote Control

- Joystick integration via joy_teleop
- Configurable button mappings
- Velocity command publishing to the rover
- Safety features (deadman switches)

### Web Visualization

- Foxglove Studio integration
- Real-time camera feed viewing
- Robot state visualization
- Interactive control interface

## Hardware Requirements

- Raspberry Pi (4 or newer recommended)
- Raspberry Pi Camera Module

## Docker Integration

The module includes a Dockerfile that:

- Builds libcamera and rpicam-apps from source
- Installs all necessary dependencies
- Sets up the ROS2 environment
- Provides an entrypoint for launching the teleop system

## Usage

### Building

```bash
cd modules/teleop
docker build -t rover0-teleop .
```

### Running

```bash
# Basic operation
docker run --device=/dev/video0:/dev/video0 --device=/dev/input/js0:/dev/input/js0 -p 8765:8765 rover0-teleop

# With custom parameters
docker run --device=/dev/video0:/dev/video0 -p 8765:8765 -e FOXGLOVE_WS_PORT=8765 rover0-teleop
```

### Accessing the Interface

1. Open Foxglove Studio (https://studio.foxglove.dev/)
2. Connect to WebSocket: `ws://<rover-ip>:8765`
3. Create a layout with:
   - Camera view panel
   - Robot state panel
   - Teleop controls panel

### Configuration

Key parameters that can be configured:

- `camera_params_file`: Camera configuration parameters
- `camera_calibration_file`: Camera calibration data
- `foxglove_bridge_port`: WebSocket server port

## Development

### Prerequisites

- ROS2 Jazzy
- libcamera development environment
- Foxglove Bridge

### Building from Source

```bash
# Clone the repository
git clone https://github.com/ar90n/rover0.git

# Build the packages
cd rover0/modules/teleop
colcon build
```

### Camera Calibration

For accurate visualization, the camera should be calibrated:

1. Use the ROS2 camera_calibration package
2. Save the calibration file to `config/ov5647_calibration.yaml`
3. Update the launch file to use the new calibration

## Integration with Other Modules

The Teleop module integrates with:

- **Rover0 Core Module**: Sends velocity commands for robot control
- **LiDAR Module**: Visualizes LiDAR data in the web interface

## Troubleshooting

### Common Issues

1. **Camera not detected**: Ensure the camera is properly connected and the correct device is passed to Docker
2. **Joystick not working**: Check the joystick device path and permissions
3. **Streaming issues**: Adjust video compression parameters for your network conditions
4. **Connection problems**: Verify network settings and firewall configurations

## License

This module is part of the Rover0 project and is licensed under the MIT License. See the LICENSE file in the root directory for details.
