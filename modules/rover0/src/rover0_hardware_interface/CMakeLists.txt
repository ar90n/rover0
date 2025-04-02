cmake_minimum_required(VERSION 3.8)
project(rover0_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Specify C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(MODULE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../.. CACHE PATH "Path to module directory")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Add common library
set(COMMON_DIR ${MODULE_DIR}/common CACHE PATH "Path to common library")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(${COMMON_DIR} ${CMAKE_BINARY_DIR}/common)

# Set the libraries to link before ament_auto_add_library
set(${PROJECT_NAME}_LIBRARIES common)

# Build your ROS 2 library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/rover0_hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME} 
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${COMMON_DIR}/include
)

pluginlib_export_plugin_description_file(hardware_interface rover0_hardware_interface.xml)

ament_auto_package()
