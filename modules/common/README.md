# Common Module

This module provides a collection of shared utilities and abstractions used across the Rover0 platform. It serves as a foundation library for hardware interfaces, communication protocols, and data structures.

## Overview

The common module is designed to:

- Abstract hardware interfaces for the Raspberry Pi Pico
- Provide communication protocols between microcontrollers
- Define message formats for inter-module communication
- Implement utility functions for data manipulation
- Offer reusable components for all Rover0 modules

## Components

### Hardware Abstractions

- **GPIO Interface (`gpio.hpp`)**: Template-based GPIO abstraction with support for:
  - Digital input/output
  - PWM output with configurable parameters
  - Type-safe pin configuration

- **UART Interface (`uart.hpp`)**: UART communication abstraction with:
  - Interrupt-driven reception
  - Buffered I/O
  - Configurable baud rates
  - Thread-safe operation

### Communication Protocols

- **Transport Layer (`transport.hpp`)**: Reliable communication protocol with:
  - Packet framing
  - Error detection
  - Serialization/deserialization
  - Asynchronous operation

- **Intercore Communication (`buffered_intercore_fifo.hpp`)**: Communication between RP2040 cores with:
  - Buffered message passing
  - Non-blocking operation
  - Fixed-size message format

### Message Definitions

- **Message Format (`message.hpp`)**: Structured message definitions for:
  - Motor control commands
  - Encoder feedback
  - IMU sensor data
  - Serialization/deserialization utilities

### Utilities

- **Endian Utilities (`endian_utils.hpp`)**: Functions for handling endianness:
  - Big-endian to little-endian conversion
  - Network byte order handling
  - Multi-byte packing/unpacking

- **Queue Implementation (`queue.hpp`)**: Thread-safe fixed-size queue with:
  - Bounded memory usage
  - Optional value semantics
  - FIFO behavior

- **Metaprogramming Utilities (`mp.hpp`)**: Template metaprogramming tools for:
  - Compile-time type checking
  - Type traits
  - Static assertions

## Usage Examples

### GPIO Control

```cpp
// Create a GPIO output on pin 25
using LedPin = Gpio<25, GPIO_FUNC_SIO, GPIO_OUT>;
auto& led = LedPin::instance();

// Toggle LED
led.write(true);   // Turn on
led.write(false);  // Turn off

// Create a PWM output on pin 7
using PwmPin = Gpio<7, GPIO_FUNC_PWM>;
auto& pwm = PwmPin::instance();

// Set PWM duty cycle (0.0 to 1.0)
pwm.write(0.5f);  // 50% duty cycle
```

### UART Communication

```cpp
// Define UART with TX on pin 0, RX on pin 1, buffer size 256
using SerialPort = Uart<0, 0, 1, 256>;
auto& uart = SerialPort::instance();

// Initialize with baud rate
uart.init(115200);

// Send data
uint8_t data[] = {0x01, 0x02, 0x03};
uart.write(data, sizeof(data));

// Read data
if (uart.has_data()) {
    uint8_t byte = uart.read();
    // Process byte
}
```

### Message Passing

```cpp
// Create a motor control message
message::MotorMsg msg{
    .param = message::MotorDevice::REAR_LEFT,
    .value = 100  // Speed value
};

// Serialize to raw data
uint32_t raw_data = message::serialize(msg);

// Send over transport
transport::send(raw_data);

// On receiving side
std::optional<uint32_t> received = transport::consume(byte);
if (received) {
    auto msg_opt = message::parse_rx_msg(received.value());
    if (msg_opt) {
        // Process message
        std::visit([](auto&& msg) {
            // Handle different message types
        }, msg_opt.value());
    }
}
```

## Building

The common module is designed to be included in other modules, but can also be built and tested independently:

```bash
mkdir build && cd build
cmake ..
make
```

## Testing

The module includes unit tests for various components:

```bash
# Build and run tests
cd build
make
./common-test
```

## Integration

To use this module in another project, add the following to your CMakeLists.txt:

```cmake
add_subdirectory(path/to/common)
target_link_libraries(your_target PRIVATE common)
```

## License

This module is part of the Rover0 project and is licensed under the MIT License. See the LICENSE file in the root directory for details.