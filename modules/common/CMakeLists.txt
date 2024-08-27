cmake_minimum_required(VERSION 3.12)
project(common VERSION 1.0.0 LANGUAGES CXX)

# Create an INTERFACE library
add_library(${PROJECT_NAME} INTERFACE)

# Specify include directories for users of this library
target_include_directories(${PROJECT_NAME} 
    INTERFACE 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Export the target
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
