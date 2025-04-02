cmake_minimum_required(VERSION 3.8)
project(rover0_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Specify C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Build your ROS 2 library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/rover0_controller.cpp
  src/wheel_odometry.cpp
  src/mecanum_kinematics.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${COMMON_DIR}/include
)

pluginlib_export_plugin_description_file(controller_interface rover0_controller.xml)

ament_auto_package()
