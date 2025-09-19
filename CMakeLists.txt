cmake_minimum_required(VERSION 3.10)
project(vcu_main)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
)

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

target_link_libraries(vcu_main
    ${GTEST_LIBRARIES}
    pthread
)

add_executable(vcu_test
    tests/simple_test.cpp
)

target_link_libraries(vcu_test
    ${GTEST_LIBRARIES}
    pthread
)

