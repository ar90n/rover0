cmake_minimum_required(VERSION 3.28)

# Include the Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Initialize the Pico SDK
pico_sdk_init()

project(rover0-lidar)


# Set the micro-ROS path
set(MICRO_ROS_PATH $ENV{MICRO_ROS_PATH})

# Set the source directory
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)


# Add micro-ROS include directory
include_directories(${MICRO_ROS_PATH}/include)

# Define the executable target, including C, C++, and ASM files from the source directory
add_executable(rover0-lidar
    ${SRC_DIR}/main.cpp
    ${SRC_DIR}/xv11lidar.cpp
)

# Link the target with the Pico standard library and micro-ROS
target_link_libraries(rover0-lidar pico_stdlib hardware_pwm hardware_uart ${MICRO_ROS_PATH}/libmicroros.a)

# Set target properties for the rover0-lidar executable
set_target_properties(rover0-lidar PROPERTIES
    C_STANDARD 11
    CXX_STANDARD 20
)

# Specify the path to the OpenOCD executable
set(OPENOCD_EXECUTABLE /usr/local/bin/openocd)

# Specify the OpenOCD interface and target configuration files
set(OPENOCD_INTERFACE_CFG cmsis-dap.cfg)
set(OPENOCD_TARGET_CFG rp2040.cfg)

# Add a custom target for flashing the RP2040
add_custom_target(flash
    COMMAND ${OPENOCD_EXECUTABLE} -f interface/${OPENOCD_INTERFACE_CFG} -f target/${OPENOCD_TARGET_CFG} -c "adapter speed 5000" -c "program ${CMAKE_BINARY_DIR}/rover0-lidar.elf verify reset exit"
    DEPENDS rover0-lidar
    COMMENT "Flashing the RP2040 with OpenOCD"
)

# Ensure the binary is built before flashing
add_dependencies(flash rover0-lidar)
