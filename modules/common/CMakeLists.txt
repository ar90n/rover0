cmake_minimum_required(VERSION 3.12)
project(common VERSION 1.0.0 LANGUAGES CXX)

# Set the source directory
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test)

# Create a static library
add_library(${PROJECT_NAME} STATIC
    ${SRC_DIR}/endian_utils.cpp
    ${SRC_DIR}/message.cpp
    ${SRC_DIR}/transport.cpp
)

# Specify include directories for this library and its users
target_include_directories(${PROJECT_NAME} 
    PUBLIC 
        $<BUILD_INTERFACE:${INCLUDE_DIR}>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${SRC_DIR}
)

# Set C++ standard
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Export the target
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Optionally, you can install the library and headers
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)


if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  # Add test executable
  add_executable(${PROJECT_NAME}-test
      ${TEST_DIR}/main.cpp
      ${TEST_DIR}/endian_utils_test.cpp
      ${TEST_DIR}/message_test.cpp
      ${TEST_DIR}/queue_test.cpp
  )
  
  # Link with the library
  target_link_libraries(${PROJECT_NAME}-test PRIVATE ${PROJECT_NAME})
  
  # Include directories for tests
  target_include_directories(${PROJECT_NAME}-test PRIVATE ${INCLUDE_DIR} ${TEST_DIR})
  
  # Set C++ standard for tests
  set_target_properties(${PROJECT_NAME}-test PROPERTIES
      CXX_STANDARD 20
      CXX_STANDARD_REQUIRED ON
  )
endif()
