cmake_minimum_required(VERSION 3.10)
project(vcu_main)

set(CMAKE_CXX_STANDARD 17)

# 选项：是否禁用libmpc
option(WITHOUT_LIBMPC "Build without libmpc++ support" OFF)

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

# 查找libmpc++（如果未禁用）
set(LIBMPC_FOUND FALSE)
if(NOT WITHOUT_LIBMPC)
    # 尝试使用pkg-config查找libmpc++
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(PC_LIBMPC QUIET mpc++)
        if(PC_LIBMPC_FOUND)
            set(LIBMPC_FOUND TRUE)
            set(LIBMPC_INCLUDE_DIRS ${PC_LIBMPC_INCLUDE_DIRS})
            set(LIBMPC_LIBRARIES ${PC_LIBMPC_LIBRARIES})
            message(STATUS "Found libmpc++ using pkg-config: ${PC_LIBMPC_VERSION}")
        endif()
    endif()
    
    # 如果pkg-config失败，手动查找
    if(NOT LIBMPC_FOUND)
        find_path(LIBMPC_INCLUDE_DIR 
            NAMES mpc/NLMPC.hpp mpc/LMPC.hpp
            PATHS 
                /usr/include
                /usr/local/include
                /opt/libmpc/include
                ${CMAKE_SOURCE_DIR}/third_party/libmpc/include
        )
        
        if(LIBMPC_INCLUDE_DIR)
            set(LIBMPC_FOUND TRUE)
            set(LIBMPC_INCLUDE_DIRS ${LIBMPC_INCLUDE_DIR})
            message(STATUS "Found libmpc++ at: ${LIBMPC_INCLUDE_DIR}")
            add_definitions(-DWITH_LIBMPC)
        else()
            message(STATUS "libmpc++ not found, building without MPC support")
        endif()
    endif()
else()
    message(STATUS "libmpc++ support disabled by WITHOUT_LIBMPC option")
endif()

# 查找其他依赖
find_package(GTest REQUIRED)

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

# 收集所有需要链接的库
set(VCU_LIBRARIES pthread)

# 添加Eigen3库
if(TARGET ${EIGEN3_TARGET})
    list(APPEND VCU_LIBRARIES ${EIGEN3_TARGET})
endif()

# 添加GTest库
list(APPEND VCU_LIBRARIES ${GTEST_LIBRARIES})

# 一次性链接所有库到主程序
target_link_libraries(vcu_main PRIVATE ${VCU_LIBRARIES})

# 系统测试
add_executable(system_test
    tests/simple_system_test.cpp
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

# 一次性链接所有库到测试程序
target_link_libraries(system_test PRIVATE pthread)

# 确保包含system_integration源文件
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/system_integration.cpp")
    message(STATUS "Creating system_integration.cpp")
endif()

