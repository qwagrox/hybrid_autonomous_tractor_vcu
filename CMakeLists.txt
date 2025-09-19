cmake_minimum_required(VERSION 3.10)
project(vcu_main)

set(CMAKE_CXX_STANDARD 17)

# 查找Eigen3
set(EIGEN3_FOUND FALSE)

# 方法1: 尝试使用find_package
find_package(Eigen3 3.3 QUIET NO_MODULE)
if(Eigen3_FOUND)
    message(STATUS "Found Eigen3 using find_package: ${Eigen3_VERSION}")
    set(EIGEN3_FOUND TRUE)
    set(EIGEN3_TARGET Eigen3::Eigen)
endif()

# 方法2: 尝试使用pkg-config
if(NOT EIGEN3_FOUND)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(PC_EIGEN3 QUIET eigen3)
        if(PC_EIGEN3_FOUND)
            message(STATUS "Found Eigen3 using pkg-config: ${PC_EIGEN3_VERSION}")
            set(EIGEN3_FOUND TRUE)
            set(EIGEN3_INCLUDE_DIRS ${PC_EIGEN3_INCLUDE_DIRS})
        endif()
    endif()
endif()

# 方法3: 直接查找标准路径
if(NOT EIGEN3_FOUND)
    set(EIGEN3_SEARCH_PATHS
        "/usr/include/eigen3"
        "/usr/local/include/eigen3"
        "/opt/eigen3/include"
        "/opt/local/include/eigen3"
    )
    
    foreach(SEARCH_PATH ${EIGEN3_SEARCH_PATHS})
        if(EXISTS "${SEARCH_PATH}/Eigen/Dense")
            message(STATUS "Found Eigen3 at: ${SEARCH_PATH}")
            set(EIGEN3_FOUND TRUE)
            set(EIGEN3_INCLUDE_DIRS ${SEARCH_PATH})
            break()
        endif()
    endforeach()
endif()

if(NOT EIGEN3_FOUND)
    message(FATAL_ERROR "Eigen3 not found! Please install libeigen3-dev.")
endif()

# 查找预编译的ACADO Toolkit
set(ACADO_FOUND FALSE)

# 查找ACADO头文件的多个可能位置
set(ACADO_SEARCH_PATHS
    "/usr/include"
    "/usr/local/include" 
    "/opt/acado/include"
    "/opt/local/include"
)

foreach(SEARCH_PATH ${ACADO_SEARCH_PATHS})
    if(EXISTS "${SEARCH_PATH}/acado_toolkit.hpp")
        message(STATUS "Found ACADO headers at: ${SEARCH_PATH}")
        set(ACADO_FOUND TRUE)
        set(ACADO_INCLUDE_DIRS ${SEARCH_PATH})
        break()
    endif()
endforeach()

# 查找ACADO库文件
if(ACADO_FOUND)
    set(ACADO_LIBRARY_PATHS
        "/usr/lib"
        "/usr/local/lib"
        "/opt/acado/lib"
        "/opt/local/lib"
    )
    
    find_library(ACADO_LIBRARY
        NAMES acado_toolkit acado libacado
        PATHS ${ACADO_LIBRARY_PATHS}
        PATH_SUFFIXES x86_64-linux-gnu
    )
    
    if(ACADO_LIBRARY)
        set(ACADO_LIBRARIES ${ACADO_LIBRARY})
        message(STATUS "Found ACADO library: ${ACADO_LIBRARY}")
    else()
        message(STATUS "ACADO library not found, using header-only mode")
        set(ACADO_LIBRARIES "")
    endif()
endif()

if(ACADO_FOUND)
    message(STATUS "ACADO Toolkit found and will be used")
    add_definitions(-DWITH_ACADO)
else()
    message(WARNING "ACADO Toolkit not found. Building without ACADO support.")
endif()

# 查找其他依赖
find_package(GTest REQUIRED)

# 设置包含目录
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

if(ACADO_FOUND)
    include_directories(${ACADO_INCLUDE_DIRS})
endif()

# 主程序
add_executable(vcu_main
    src/main_vcu_system.cpp
    src/can_bus_interface.cpp
    src/control/braking_controller.cpp
    src/control/cvt_controller.cpp
    src/control/energy_manager.cpp
    src/control/implement_control_manager.cpp
    src/control/implement_drivers/fertilizer_controller.cpp
    src/control/implement_drivers/plow_controller.cpp
    src/control/implement_drivers/seeder_controller.cpp
    src/control/implement_drivers/sprayer_controller.cpp
    src/control/steering_controller.cpp
    src/control/torque_arbiter.cpp
    src/diagnostic/adaptive_learner.cpp
    src/diagnostic/data_logger.cpp
    src/diagnostic/health_monitor.cpp
    src/execution/actuator_interface.cpp
    src/execution/fault_handler.cpp
    src/hardware/watchdog.cpp
    src/models/battery_model.cpp
    src/models/engine_model.cpp
    src/models/motor_model.cpp
    src/perception/load_detector.cpp
    src/perception/sensor_fusion.cpp
    src/prediction/predictive_analytics.cpp
    src/system_integration.cpp
    src/utils/tractor_state_calculator.cpp
)

# 链接库
if(TARGET ${EIGEN3_TARGET})
    target_link_libraries(vcu_main PRIVATE ${EIGEN3_TARGET})
endif()

target_link_libraries(vcu_main
    ${GTEST_LIBRARIES}
    pthread
)

if(ACADO_FOUND AND ACADO_LIBRARIES)
    target_link_libraries(vcu_main ${ACADO_LIBRARIES})
endif()

# 系统测试
add_executable(system_test
    tests/system_test.cpp
    src/can_bus_interface.cpp
    src/control/braking_controller.cpp
    src/control/cvt_controller.cpp
    src/control/energy_manager.cpp
    src/control/implement_control_manager.cpp
    src/control/implement_drivers/fertilizer_controller.cpp
    src/control/implement_drivers/plow_controller.cpp
    src/control/implement_drivers/seeder_controller.cpp
    src/control/implement_drivers/sprayer_controller.cpp
    src/control/steering_controller.cpp
    src/control/torque_arbiter.cpp
    src/diagnostic/adaptive_learner.cpp
    src/diagnostic/data_logger.cpp
    src/diagnostic/health_monitor.cpp
    src/execution/actuator_interface.cpp
    src/execution/fault_handler.cpp
    src/hardware/watchdog.cpp
    src/models/battery_model.cpp
    src/models/engine_model.cpp
    src/models/motor_model.cpp
    src/perception/load_detector.cpp
    src/perception/sensor_fusion.cpp
    src/prediction/predictive_analytics.cpp
    src/system_integration.cpp
    src/utils/tractor_state_calculator.cpp
)

if(TARGET ${EIGEN3_TARGET})
    target_link_libraries(system_test PRIVATE ${EIGEN3_TARGET})
endif()

target_link_libraries(system_test
    pthread
)

if(ACADO_FOUND AND ACADO_LIBRARIES)
    target_link_libraries(system_test ${ACADO_LIBRARIES})
endif()
