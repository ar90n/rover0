# Rover0

A ROS2-based autonomous rover platform with navigation capabilities.

<!-- Add a rover image here -->
<!-- Example: ![Rover0](https://github.com/ar90n/rover0/raw/main/docs/images/rover0.jpg) -->

## Overview

Rover0 is a DIY rover platform built with ROS2 (Robot Operating System 2) that integrates:

- Differential drive motor control
- LiDAR-based mapping and navigation
- IMU-based localization
- Teleoperation capabilities
- Autonomous navigation using Nav2

The system is designed to run on a Raspberry Pi with hardware interfaces for motor control and sensor integration.

## Hardware Components

- Raspberry Pi (main controller)
- Raspberry Pi Pico (motor controller)
- XV11 LiDAR sensor
- IMU sensor
- Differential drive motors with encoders
- Custom motor driver board

## Software Architecture

The project is organized into several modules:

- **rover0**: Main rover control and ROS2 integration
- **motor**: Motor control firmware for Raspberry Pi Pico
- **lidar**: LiDAR sensor interface
- **teleop**: Teleoperation interface
- **uros-agent-lidar**: Micro-ROS agent for LiDAR communication

## Getting Started

### Prerequisites

- ROS2 (Humble or later)
- Docker and Docker Compose
- Raspberry Pi with Ubuntu 22.04 or later
- Raspberry Pi Pico with Micro-ROS firmware

### Environment Setup

1. Clone this repository:
   ```bash
   git clone https://github.com/ar90n/rover0.git
   cd rover0
   ```

2. Set up environment variables:
   ```bash
   export MOTOR_SERIAL_PORT=/dev/ttyACM0  # Adjust as needed
   export LIDAR_SERIAL_PORT=/dev/ttyUSB0  # Adjust as needed
   export FOXGLOVE_WS_PORT=8765           # Optional
   ```

### Running the Rover

#### Basic Operation

```bash
docker-compose up
```

#### Development Mode

```bash
docker-compose -f docker-compose.yml -f overrides/docker-compose.override.rover0.dev.yml up
```

#### Mapping Mode

```bash
docker-compose -f docker-compose.yml -f overrides/docker-compose.override.rover0.mappning.yml up
```

#### Teleoperation

```bash
docker-compose -f docker-compose.yml -f overrides/docker-compose.override.teleop.dev.yml up
```

## Features

- **Autonomous Navigation**: Using ROS2 Nav2 stack
- **Mapping**: SLAM-based mapping capabilities
- **Teleoperation**: Remote control via web interface
- **Sensor Integration**: LiDAR and IMU data fusion
- **Simulation Support**: Gazebo simulation environment

## Development

### Project Structure

- `/modules`: Contains all ROS2 packages and modules
- `/libs`: External libraries and dependencies
- `/overrides`: Docker Compose override files for different configurations
- `/root`: System configuration files

### Building the Firmware

For the motor controller:

```bash
cd modules/motor
mkdir build && cd build
cmake ..
make
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- ROS2 community
- Nav2 project
- Micro-ROS project
