cmake_minimum_required(VERSION 3.28)
set(MODULE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/.. CACHE PATH "Path to module directory")
# Include the Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
# Initialize the Pico SDK
pico_sdk_init()
project(rover0-motor)

# Set the source directory
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
set(TEST_DIR ${CMAKE_SOURCE_DIR}/test)

# Add common library
set(COMMON_DIR ${MODULE_DIR}/common CACHE PATH "Path to common library")
add_subdirectory(${COMMON_DIR} ${CMAKE_BINARY_DIR}/common)

# Define the executable target
add_executable(rover0-motor
    ${SRC_DIR}/main.cpp
    ${SRC_DIR}/comm_proc.cpp
    ${SRC_DIR}/main_proc.cpp
    ${SRC_DIR}/transport.cpp
    ${SRC_DIR}/logger.cpp
)

# Link the target with libraries
target_link_libraries(rover0-motor 
    PRIVATE
        common::common
        pico_stdlib 
        pico_multicore 
        pico_async_context_poll 
        hardware_pwm 
        hardware_uart 
        hardware_pio 
        hardware_i2c
)

set(PICO_ASYNC_CONTEXT_POLL_DEFAULT_STACK_SIZE 4096)

# Set target properties
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
    COMMAND g++ -std=c++20 ${TEST_DIR}/main.cpp ${TEST_DIR}/queue_test.cpp -o rover0-motor-test -I ${SRC_DIR} -I ${COMMON_DIR}/include
    COMMENT "Run test"
)

# Ensure the binary is built before flashing
add_dependencies(flash rover0-motor)
