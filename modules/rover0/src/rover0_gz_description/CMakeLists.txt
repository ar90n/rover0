cmake_minimum_required(VERSION 3.8)
project(rover0_gz_description)

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_package(INSTALL_TO_SHARE urdf config)
