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

# VCU Controller Library
add_library(vcu_controller
    src/cvt/cvt_controller.cpp
    src/can/can_interface.cpp
    src/can/j1939_protocol.cpp
    src/can/socketcan_interface.cpp
    src/sensors/sensor_data_manager.cpp
    src/prediction/load_predictor.cpp
    src/core/vcu_service.cpp
    src/hal/linux_hal.cpp
    src/config/json_config_manager.cpp
    src/diag/file_diagnostic_monitor.cpp
)

target_include_directories(vcu_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Enable testing
enable_testing()

# CVT Tests
add_executable(cvt_tests
    tests/test_main.cpp
    tests/cvt/test_cvt_controller.cpp
)

target_link_libraries(cvt_tests
    vcu_controller
    GTest::gtest
    GTest::gtest_main
)

# CAN Tests
add_executable(can_tests
    tests/test_main.cpp
    tests/can/test_can_frame.cpp
    tests/can/test_j1939_protocol.cpp
    tests/can/test_can_interface.cpp
)

target_link_libraries(can_tests
    vcu_controller
    GTest::gtest
    GTest::gtest_main
)

# Sensor Tests
add_executable(sensor_tests
    tests/test_main.cpp
    tests/sensors/test_sensor_simple.cpp
)

target_link_libraries(sensor_tests
    vcu_controller
    GTest::gtest
    GTest::gtest_main
)

# Prediction Tests
add_executable(prediction_tests
    tests/test_main.cpp
    tests/prediction/test_load_predictor.cpp
)

target_link_libraries(prediction_tests
    vcu_controller
    GTest::gtest
    GTest::gtest_main
)

# Core Service Tests
add_executable(core_tests
    tests/test_main.cpp
    tests/core/test_vcu_service.cpp
    tests/config/test_config_manager.cpp
    tests/diag/test_diagnostic_monitor.cpp
)

target_link_libraries(core_tests
    vcu_controller
    GTest::gtest
    GTest::gtest_main
)

# VCU Main Executable
add_executable(vcu_main
    src/vcu_main.cpp
)

target_link_libraries(vcu_main
    vcu_controller
    pthread
)

# Add tests
add_test(NAME cvt_controller_tests COMMAND cvt_tests)
add_test(NAME can_module_tests COMMAND can_tests)
add_test(NAME sensor_module_tests COMMAND sensor_tests)
add_test(NAME prediction_module_tests COMMAND prediction_tests)
add_test(NAME core_service_tests COMMAND core_tests)

# Installation
install(TARGETS vcu_controller vcu_main
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
