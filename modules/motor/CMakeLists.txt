cmake_minimum_required(VERSION 3.28)

# Include the Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Initialize the Pico SDK
pico_sdk_init()

project(rover0-motor)

# Set the source directory
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
set(TEST_DIR ${CMAKE_SOURCE_DIR}/test)

# Define the executable target, including C, C++, and ASM files from the source directory
add_executable(rover0-motor
    ${SRC_DIR}/main.cpp
    ${SRC_DIR}/logger.cpp
    ${SRC_DIR}/transport.cpp
    ${SRC_DIR}/comm_proc.cpp
    ${SRC_DIR}/main_proc.cpp
    ${SRC_DIR}/message.cpp
    ${SRC_DIR}/endian_utils.cpp
)

# Link the target with the Pico standard library and micro-ROS
target_link_libraries(rover0-motor pico_stdlib pico_multicore pico_async_context_poll hardware_pwm hardware_uart hardware_pio hardware_i2c)

set(PICO_ASYNC_CONTEXT_POLL_DEFAULT_STACK_SIZE 4096)


# Set target properties for the rover0-motor executable
set_target_properties(rover0-motor PROPERTIES
    C_STANDARD 11
    CXX_STANDARD 20
)

# Specify the path to the OpenOCD executable
set(OPENOCD_EXECUTABLE /usr/local/bin/openocd)

# Specify the OpenOCD interface and target configuration files
set(OPENOCD_INTERFACE_CFG cmsis-dap.cfg)
set(OPENOCD_TARGET_CFG rp2040.cfg)

pico_generate_pio_header(rover0-motor ${CMAKE_CURRENT_LIST_DIR}/src/encoder.pio)
pico_enable_stdio_usb(rover0-motor 1)
pico_enable_stdio_uart(rover0-motor 0)

# Add a custom target for flashing the RP2040
add_custom_target(flash
    COMMAND ${OPENOCD_EXECUTABLE} -f interface/${OPENOCD_INTERFACE_CFG} -f target/${OPENOCD_TARGET_CFG} -c "adapter speed 5000" -c "program ${CMAKE_BINARY_DIR}/rover0-motor.elf verify reset exit"
    DEPENDS rover0-motor
    COMMENT "Flashing the RP2040 with OpenOCD"
)

add_custom_target(test
    COMMAND g++ -std=c++20  ${SRC_DIR}/message.cpp ${SRC_DIR}/endian_utils.cpp  ${TEST_DIR}/main.cpp ${TEST_DIR}/message_test.cpp ${TEST_DIR}/queue_test.cpp ${TEST_DIR}/endian_utils_test.cpp  -o rover0-motor-test -I ${SRC_DIR}
    COMMENT "Run test"
)

# Ensure the binary is built before flashing
add_dependencies(flash rover0-motor)
