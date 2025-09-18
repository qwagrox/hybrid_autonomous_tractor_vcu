cmake_minimum_required(VERSION 3.12)
project(autonomous_tractor_vcu VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖查找
find_package(Eigen3 REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(Threads REQUIRED)
find_package(GTest REQUIRED)

# 包含目录
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)

# 使用file指令自动收集所有源文件
file(GLOB_RECURSE ALL_SOURCES "src/*.cpp")

# 可执行文件
add_executable(vcu_main ${ALL_SOURCES})

# 链接库
target_link_libraries(vcu_main
    Eigen3::Eigen
    yaml-cpp
    ${CMAKE_THREAD_LIBS_INIT}
    gpiod
    # can 和其他硬件相关库可能需要根据实际情况链接
)

# 添加测试
add_executable(unit_tests tests/unit_tests/test_implement_control.cpp)
target_link_libraries(unit_tests GTest::GTest GTest::Main Gmock::Gmock)

# 安装目标
install(TARGETS vcu_main DESTINATION bin)
install(DIRECTORY config/ DESTINATION etc/vcu)
install(DIRECTORY models/ DESTINATION share/vcu/models)

