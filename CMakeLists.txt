cmake_minimum_required(VERSION 3.10)
project(vcu_main)

# 设置C++标准为20（libmpc++需要C++20）
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# 查找libmpc++（可选）
set(LIBMPC_FOUND FALSE)
option(WITHOUT_LIBMPC "Build without libmpc++ support" OFF)

if(NOT WITHOUT_LIBMPC)
    set(LIBMPC_SEARCH_PATHS
        "/usr/local/include"
        "/usr/include"
        "/opt/libmpc/include"
    )
    
    foreach(SEARCH_PATH ${LIBMPC_SEARCH_PATHS})
        if(EXISTS "${SEARCH_PATH}/mpc/NLMPC.hpp")
            message(STATUS "Found libmpc++ headers at: ${SEARCH_PATH}")
            set(LIBMPC_FOUND TRUE)
            set(LIBMPC_INCLUDE_DIRS ${SEARCH_PATH})
            add_definitions(-DWITH_LIBMPC)
            break()
        endif()
    endforeach()
endif()

if(LIBMPC_FOUND)
    message(STATUS "libmpc++ found and will be used with C++20")
else()
    message(STATUS "libmpc++ not found, building without MPC support")
endif()

# 查找其他依赖
find_package(GTest QUIET)
if(NOT GTest_FOUND)
    message(STATUS "GTest not found, building without test support")
endif()

# 设置包含目录
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

if(LIBMPC_FOUND)
    include_directories(${LIBMPC_INCLUDE_DIRS})
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
set(VCU_MAIN_LIBS pthread)

if(TARGET ${EIGEN3_TARGET})
    list(APPEND VCU_MAIN_LIBS ${EIGEN3_TARGET})
endif()

if(GTest_FOUND)
    list(APPEND VCU_MAIN_LIBS ${GTEST_LIBRARIES})
endif()

target_link_libraries(vcu_main ${VCU_MAIN_LIBS})

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

set(SYSTEM_TEST_LIBS pthread)

if(TARGET ${EIGEN3_TARGET})
    list(APPEND SYSTEM_TEST_LIBS ${EIGEN3_TARGET})
endif()

target_link_libraries(system_test ${SYSTEM_TEST_LIBS})

# 设置编译选项
target_compile_options(vcu_main PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-narrowing
)

target_compile_options(system_test PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-narrowing
)

# 输出配置信息
message(STATUS "=== VCU Build Configuration ===")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Eigen3 Found: ${EIGEN3_FOUND}")
message(STATUS "libmpc++ Found: ${LIBMPC_FOUND}")
if(LIBMPC_FOUND)
    message(STATUS "libmpc++ will use C++20 features")
endif()
message(STATUS "===============================")
