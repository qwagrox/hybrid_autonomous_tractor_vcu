# CMakeLists.txt
cmake_minimum_required(VERSION 3.12)
project(autonomous_tractor_vcu VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖查找
find_package(Eigen3 REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(Threads REQUIRED)

# 包含目录
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
)

# 可执行文件
add_executable(vcu_main
    src/main_vcu_system.cpp
    src/core/system_integration.cpp
    src/can_bus_interface.cpp
    src/perception/sensor_fusion.cpp
    src/perception/load_detector.cpp
    src/prediction/predictive_analytics.cpp
    src/control/torque_arbiter.cpp
    src/control/cvt_controller.cpp
    src/control/energy_manager.cpp
    src/execution/actuator_interface.cpp
    src/execution/safety_monitor.cpp
    src/execution/fault_handler.cpp
    src/diagnostic/health_monitor.cpp
    src/diagnostic/data_logger.cpp
    src/diagnostic/adaptive_learner.cpp
    src/models/engine_model.cpp
    src/models/motor_model.cpp
)

# 链接库
target_link_libraries(vcu_main
    Eigen3::Eigen
    yaml-cpp
    ${CMAKE_THREAD_LIBS_INIT}
    can
    gpiod
)

# 安装目标
install(TARGETS vcu_main DESTINATION bin)
install(DIRECTORY config/ DESTINATION etc/vcu)
install(DIRECTORY models/ DESTINATION share/vcu/models)