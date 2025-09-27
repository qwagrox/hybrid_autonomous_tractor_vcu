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

# Platform detection and configuration
if(NOT DEFINED TARGET_PLATFORM)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(TARGET_PLATFORM "LINUX" CACHE STRING "Target platform")
    else()
        set(TARGET_PLATFORM "LINUX" CACHE STRING "Target platform")
    endif()
endif()

# Platform-specific definitions
if(TARGET_PLATFORM STREQUAL "LINUX")
    add_definitions(-DPLATFORM_LINUX)
    message(STATUS "Building for Linux platform")
elseif(TARGET_PLATFORM STREQUAL "NUTTX")
    add_definitions(-DPLATFORM_NUTTX)
    message(STATUS "Building for NuttX platform")
else()
    message(FATAL_ERROR "Unsupported platform: ${TARGET_PLATFORM}")
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# POSIX compliance flags
add_definitions(-D_POSIX_C_SOURCE=200809L)
add_definitions(-D_GNU_SOURCE)

# Include directories
include_directories(include)

# Find required packages (Linux only)
if(TARGET_PLATFORM STREQUAL "LINUX")
    find_package(GTest REQUIRED)
endif()

# Platform abstraction layer sources
set(PLATFORM_SOURCES
    src/platform/platform_factory.cpp
)

# Platform-specific sources
if(TARGET_PLATFORM STREQUAL "LINUX")
    list(APPEND PLATFORM_SOURCES
        src/platform/linux/linux_platform.cpp
        src/platform/linux/linux_thread.cpp
        src/platform/linux/linux_mutex.cpp
        src/platform/linux/linux_condition_variable.cpp
        src/platform/linux/linux_time.cpp
    )
elseif(TARGET_PLATFORM STREQUAL "NUTTX")
    list(APPEND PLATFORM_SOURCES
        src/platform/nuttx/nuttx_platform.cpp
        src/platform/nuttx/nuttx_thread.cpp
        src/platform/nuttx/nuttx_mutex.cpp
        src/platform/nuttx/nuttx_condition_variable.cpp
        src/platform/nuttx/nuttx_time.cpp
        src/platform/nuttx/nuttx_can_interface.cpp
    )
endif()

# VCU Controller Library
add_library(vcu_controller
    # Core modules
    src/cvt/cvt_controller.cpp
    src/cvt/cvt_config.cpp
    src/cvt/cvt_strategy_factory.cpp
    src/cvt/hmcvt_vendor1_strategy.cpp
    src/can/can_interface.cpp
    src/can/j1939_protocol.cpp
    src/can/platform_can_interface.cpp
    src/sensors/platform_sensor_data_manager.cpp
    src/prediction/platform_load_predictor.cpp
    src/core/vcu_service.cpp
    src/adas_interface/adas_can_interface.cpp
    src/adas_interface/adas_interface_factory.cpp
    src/hal/linux_hal.cpp
    src/config/json_config_manager.cpp
    src/diag/file_diagnostic_monitor.cpp
    
    # Hydraulic control modules (NEW)
    src/core/hydraulic_service.cpp
    
    # Platform abstraction layer
    ${PLATFORM_SOURCES}
)

# Platform-specific sources for Linux
if(TARGET_PLATFORM STREQUAL "LINUX")
    target_sources(vcu_controller PRIVATE
        src/can/socketcan_interface.cpp
        src/sensors/sensor_data_manager.cpp
        src/prediction/load_predictor.cpp
    )
endif()

target_include_directories(vcu_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Platform-specific linking
if(TARGET_PLATFORM STREQUAL "LINUX")
    target_link_libraries(vcu_controller PUBLIC pthread)
elseif(TARGET_PLATFORM STREQUAL "NUTTX")
    # NuttX-specific libraries would be linked here
    target_link_libraries(vcu_controller PUBLIC pthread)
endif()

# VCU Main Executable
add_executable(vcu_main
    src/vcu_main.cpp
)

target_link_libraries(vcu_main
    vcu_controller
)

# Hydraulic Control Example (NEW)
add_executable(hydraulic_control_example
    examples/hydraulic_control_example.cpp
)

target_link_libraries(hydraulic_control_example
    vcu_controller
)

# Testing (Linux only for now)
if(TARGET_PLATFORM STREQUAL "LINUX")
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

    # Hydraulic Control Tests (NEW)
    add_executable(hydraulic_tests
        tests/test_main.cpp
        tests/test_hydraulic_control.cpp
    )

    target_link_libraries(hydraulic_tests
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

    # Add tests
    add_test(NAME cvt_controller_tests COMMAND cvt_tests)
    add_test(NAME can_module_tests COMMAND can_tests)
    add_test(NAME hydraulic_control_tests COMMAND hydraulic_tests)  # NEW
    add_test(NAME prediction_module_tests COMMAND prediction_tests)
    add_test(NAME core_service_tests COMMAND core_tests)
endif()

# Installation
install(TARGETS vcu_controller vcu_main hydraulic_control_example
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

# Print configuration summary
message(STATUS "=== VCU Controller Build Configuration ===")
message(STATUS "Target Platform: ${TARGET_PLATFORM}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Install Prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Hydraulic Control: ENABLED")  # NEW
message(STATUS "==========================================")
