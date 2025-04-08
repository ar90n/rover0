# Motor Controller Module

This module contains the firmware for the Raspberry Pi Pico microcontroller that handles motor control, encoder feedback, and IMU sensor data for the Rover0 platform.

## Overview

The motor controller module is responsible for:

- Controlling four DC motors (front-left, front-right, rear-left, rear-right)
- Reading and processing encoder feedback for precise motor control
- Reading IMU (Inertial Measurement Unit) data for orientation sensing
- Implementing PID control for accurate motor speed regulation
- Handling emergency stop functionality
- Communicating with the main Raspberry Pi via UART

## Hardware Components

- **Microcontroller**: Raspberry Pi Pico (RP2040)
- **Motors**: 4 DC motors with encoders (differential drive configuration)
- **IMU Sensor**: MPU-6050 (accelerometer and gyroscope)
- **Motor Drivers**: H-bridge motor drivers
- **Emergency Switch**: Hardware emergency stop

## Pin Configuration

The pin configuration is defined in `config.hpp`:

| Component | Pin(s) |
|-----------|--------|
| LED | 25 |
| Emergency Switch | 28 |
| Rear Left Motor Forward | 4 |
| Rear Left Motor Backward | 5 |
| Rear Left Encoder | 0 |
| Rear Right Motor Forward | 2 |
| Rear Right Motor Backward | 3 |
| Rear Right Encoder | 1 |
| Front Left Motor Forward | 10 |
| Front Left Motor Backward | 11 |
| Front Left Encoder | 15 |
| Front Right Motor Forward | 12 |
| Front Right Motor Backward | 13 |
| Front Right Encoder | 14 |
| UART TX (to Raspberry Pi) | 16 |
| UART RX (from Raspberry Pi) | 17 |
| I2C SDA (for IMU) | 20 |
| I2C SCL (for IMU) | 21 |

## Software Architecture

The firmware is organized into several components:

- **Main Process (`main_proc.cpp`)**: Initializes hardware and runs the main control loop
- **Communication Process (`comm_proc.cpp`)**: Handles UART communication with the Raspberry Pi
- **Motor Driver (`motor_driver.cpp`)**: Implements PID control for motor speed regulation
- **IMU Interface (`imu.hpp`)**: Interfaces with the MPU-6050 IMU sensor
- **Message Handling**: Processes commands from the Raspberry Pi and sends back sensor data

### Control System

The motor controller implements a PID (Proportional-Integral-Derivative) control system for precise motor speed regulation. The PID parameters are configurable in `config.hpp`:

```cpp
static constexpr float MOTOR_DRIVER_PID_KP = 0.100;
static constexpr float MOTOR_DRIVER_PID_KI = 0.085;
static constexpr float MOTOR_DRIVER_PID_KD = 0.020;
```

### Task Scheduling

The firmware uses a cooperative multitasking approach with several periodic tasks:

- Motor control: Updates motor speeds based on PID calculations (10ms interval)
- IMU reading: Reads accelerometer and gyroscope data (20ms interval)
- Emergency check: Monitors emergency switch state (10ms interval)
- LED heartbeat: Toggles LED for visual feedback (500ms interval)

## Building the Firmware

### Prerequisites

- CMake (3.12 or later)
- Raspberry Pi Pico SDK
- GCC ARM toolchain

### Build Instructions

1. Clone the repository and navigate to the motor module:
   ```bash
   cd modules/motor
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

4. The compiled firmware will be available as `motor.uf2` in the build directory.

### Flashing the Firmware

1. Connect the Raspberry Pi Pico to your computer while holding the BOOTSEL button.
2. Release the button after connecting.
3. The Pico will appear as a mass storage device.
4. Copy the `motor.uf2` file to the Pico.
5. The Pico will automatically reboot and run the firmware.

## Hardware Schematics

The hardware schematics for the motor controller board are available in the `schematics.pdf` file in this directory.

## Testing

The `test` directory contains unit tests for various components of the firmware. To run the tests:

```bash
cd test
mkdir build && cd build
cmake ..
make
./motor_test
```

## Calibration

The IMU sensor may require calibration for accurate readings. The `tool/calibrate_imu` directory contains a utility for calibrating the IMU sensor.

## Troubleshooting

### Common Issues

1. **Motors not responding**: Check the motor connections and ensure the emergency switch is not activated.
2. **Erratic motor behavior**: Verify the PID parameters in `config.hpp` and adjust if necessary.
3. **Communication issues**: Check the UART connections and baud rate settings.
4. **IMU data errors**: Ensure the I2C connections are correct and the IMU is properly initialized.

## License

This module is part of the Rover0 project and is licensed under the MIT License. See the LICENSE file in the root directory for details.
