# Rover0 Core Module

This module contains the core ROS2 packages for the Rover0 platform, providing the main robot functionality including hardware interfaces, controllers, and navigation capabilities.

## Overview

The Rover0 Core module is responsible for:

- Interfacing with the robot's hardware (motors, encoders, IMU)
- Implementing ROS2 control framework for the rover
- Providing robot description (URDF) for visualization and simulation
- Handling navigation and autonomous movement
- Managing the robot's state and lifecycle

## Components

### ROS2 Packages

- **rover0_bringup**: Launch files and configurations for starting the rover
- **rover0_controller**: Custom controllers for the rover's movement
- **rover0_description**: URDF models and meshes for visualization
- **rover0_gz_description**: Gazebo-specific robot description for simulation
- **rover0_hardware_interface**: Hardware abstraction layer for motors and sensors
- **rover0_robot_description**: Physical robot-specific description and parameters

## Architecture

The module follows the standard ROS2 control architecture:

1. **Hardware Interface Layer**: Communicates with physical hardware (motors, encoders, IMU)
2. **Controller Layer**: Implements control algorithms for the robot's movement
3. **Navigation Layer**: Provides path planning and autonomous navigation
4. **Description Layer**: Defines the robot's physical properties and appearance

## Hardware Interface

The hardware interface connects to:

- Motor controllers via serial communication
- IMU sensor for orientation data
- Encoders for odometry and position tracking

The interface is implemented using the ROS2 `ros2_control` framework, which provides a standardized way to interact with hardware.

## Navigation

The module integrates with the Nav2 stack for autonomous navigation, including:

- Path planning
- Obstacle avoidance
- Localization
- Mapping

## Docker Integration

The module includes a Dockerfile for containerized deployment, which:

- Builds all ROS2 packages
- Installs necessary dependencies
- Provides an entrypoint for launching the rover

## Usage

### Building

```bash
cd modules/rover0
docker build -t rover0-core .
```

### Running

```bash
# Basic operation
docker run --device=/dev/ttyACM0:/dev/ttyACM0 rover0-core

# With custom parameters
docker run --device=/dev/ttyACM0:/dev/ttyACM0 rover0-core rover0.launch.py use_sim:=false use_rviz:=true
```

### Launch Files

The main launch files are:

- `rover0.launch.py`: Standard operation mode
- `rover0.mapping.launch.py`: Mapping mode for creating new maps

### Parameters

Key parameters that can be configured:

- `use_sim`: Enable/disable simulation mode
- `use_rviz`: Enable/disable visualization
- `map`: Path to the map file for navigation
- `nav2_params_file`: Navigation parameters

## Development

### Prerequisites

- ROS2 Jazzy
- ros2_control
- navigation2
- robot_localization

### Building from Source

```bash
# Clone the repository
git clone https://github.com/ar90n/rover0.git

# Build the packages
cd rover0/modules/rover0
colcon build
```

### Testing

```bash
# Run tests
colcon test
```

## Integration with Other Modules

The Rover0 Core module integrates with:

- **LiDAR Module**: For obstacle detection and mapping
- **Teleop Module**: For remote control
- **Common Module**: For shared utilities and abstractions

## License

This module is part of the Rover0 project and is licensed under the MIT License. See the LICENSE file in the root directory for details.