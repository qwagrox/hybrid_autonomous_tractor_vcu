cmake_minimum_required(VERSION 3.12)
project(vcu_controller VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Include directories
include_directories(include)

# Find required packages
find_package(GTest REQUIRED)

# CVT Controller Library
add_library(cvt_controller
    src/cvt/cvt_controller.cpp
)

target_include_directories(cvt_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Enable testing
enable_testing()

# Test executable
add_executable(cvt_tests
    tests/test_main.cpp
    tests/cvt/test_cvt_controller.cpp
)

target_link_libraries(cvt_tests
    cvt_controller
    GTest::gtest
    GTest::gtest_main
)

# Add test
add_test(NAME cvt_controller_tests COMMAND cvt_tests)

# Installation
install(TARGETS cvt_controller
    EXPORT vcu_controller_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

# Export targets
install(EXPORT vcu_controller_targets
    FILE vcu_controller_targets.cmake
    NAMESPACE vcu::
    DESTINATION lib/cmake/vcu_controller
)

# Create config file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/vcu_controller_config_version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/vcu_controller_config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/vcu_controller_config.cmake"
    INSTALL_DESTINATION lib/cmake/vcu_controller
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/vcu_controller_config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/vcu_controller_config_version.cmake"
    DESTINATION lib/cmake/vcu_controller
)
