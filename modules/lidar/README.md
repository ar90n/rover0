# LiDAR Module

This module contains the firmware for interfacing with the XV11 LiDAR sensor on the Rover0 platform, providing 360-degree distance measurements for mapping and navigation.

## Overview

The LiDAR module is responsible for:

- Controlling the XV11 LiDAR motor speed
- Reading and processing LiDAR distance data
- Publishing ROS2 LaserScan messages
- Communicating with the main Raspberry Pi via Micro-ROS

## Hardware Components

- **LiDAR Sensor**: Neato XV11 LiDAR
- **Microcontroller**: Raspberry Pi Pico (RP2040)
- **Motor Control**: PWM-based motor speed control
- **Communication**: UART for LiDAR data and Micro-ROS communication

## Pin Configuration

The pin configuration is defined in `config.hpp`:

| Component | Pin(s) |
|-----------|--------|
| LED | 25 |
| PWM (Motor Control) | 7 |
| UART LiDAR TX | 0 |
| UART LiDAR RX | 1 |
| UART Control TX | 4 |
| UART Control RX | 5 |

## Software Architecture

The firmware is organized into several components:

- **Main Process (`main.cpp`)**: Initializes hardware, sets up ROS2 nodes, and manages the main control loop
- **Communication Process (`comm.cpp`)**: Runs on Core1 to handle LiDAR data acquisition and motor control
- **XV11 LiDAR Interface (`xv11lidar.cpp`, `xv11lidar.h`)**: Interfaces with the XV11 LiDAR sensor
- **Task Scheduling**: Uses cooperative multitasking for different operations

### Dual-Core Operation

The firmware utilizes both cores of the RP2040 microcontroller:

- **Core0**: Handles ROS2 communication and publishes LaserScan messages
- **Core1**: Manages LiDAR data acquisition, motor control, and preprocessing

### LiDAR Data Processing

The XV11 LiDAR provides distance measurements in 4-degree chunks. The firmware:

1. Reads raw data packets from the LiDAR
2. Validates packet checksums
3. Extracts distance measurements
4. Converts to ROS2 LaserScan format with 1-degree resolution
5. Publishes data to the ROS2 network

### Motor Control

The LiDAR motor speed is controlled using a PI (Proportional-Integral) controller to maintain a consistent rotation rate:

```cpp
PIDType m_motor_pid = mamePID::pi<float>(1e-4, 1e-6, LIDAR_SAMPLE_TIME_MS, -1.0, 1.0);
```

The target rotation speed is set to 270 RPM by default, which can be configured in `config.hpp`.

## ROS2 Integration

The module publishes LiDAR data as ROS2 LaserScan messages:

- **Topic**: `/scan`
- **Frame ID**: `rover0_lidar_link`
- **Message Type**: `sensor_msgs/LaserScan`
- **Angular Range**: -π to π radians (full 360 degrees)
- **Range Limits**: 0.15m to 6.0m

## Building the Firmware

### Prerequisites

- CMake (3.12 or later)
- Raspberry Pi Pico SDK
- GCC ARM toolchain
- Micro-ROS libraries

### Build Instructions

1. Clone the repository and navigate to the lidar module:
   ```bash
   cd modules/lidar
   ```

2. Create a build directory and navigate to it:
   ```bash
   mkdir build && cd build
   ```

3. Configure and build the project:
   ```bash
   cmake ..
   make
   ```

4. The compiled firmware will be available as `lidar.uf2` in the build directory.

### Flashing the Firmware

1. Connect the Raspberry Pi Pico to your computer while holding the BOOTSEL button.
2. Release the button after connecting.
3. The Pico will appear as a mass storage device.
4. Copy the `lidar.uf2` file to the Pico.
5. The Pico will automatically reboot and run the firmware.

## Hardware Schematics

The hardware schematics for the LiDAR interface board are available in the `schematics.pdf` file in this directory.

## Testing

The `test` directory contains unit tests for various components of the firmware. To run the tests:

```bash
cd test
mkdir build && cd build
cmake ..
make
./lidar_test
```

## Troubleshooting

### Common Issues

1. **LiDAR not spinning**: Check the PWM connection and ensure the motor is receiving power.
2. **No data received**: Verify the UART connections and baud rate settings.
3. **Inconsistent readings**: Check for reflective surfaces or direct sunlight that might interfere with the LiDAR.
4. **Motor speed fluctuations**: Adjust the PI controller parameters if needed.

### Debugging

The LED on pin 25 blinks to indicate that the firmware is running. The blink rate is controlled by the heartbeat task in `comm.cpp`.

## License

The XV11 LiDAR interface code is based on:
- Copyright 2018 (C) Bartosz Meglicki <meglickib@gmail.com>
- Copyright 2024 (C) Masahiro Wada <argon.argon.argon@gmail.com>

This module is part of the Rover0 project and is licensed under the Mozilla Public License, v. 2.0. See the LICENSE file in the root directory for details.