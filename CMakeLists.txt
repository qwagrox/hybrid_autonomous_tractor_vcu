cmake_minimum_required(VERSION 3.16)
project(HybridAutonomousTractorVCU)

# 设置C++标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable -Wno-narrowing")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# 包含目录
include_directories(include)
include_directories(third_party)  # 包含内嵌的libmpc++

# 查找Eigen3
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
    message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")
    include_directories(${EIGEN3_INCLUDE_DIR})
else()
    message(FATAL_ERROR "Eigen3 not found")
endif()

# 查找NLopt (libmpc++依赖)
find_package(PkgConfig QUIET)
set(NLOPT_FOUND FALSE)
if(PkgConfig_FOUND)
    pkg_check_modules(NLOPT QUIET nlopt)
    if(NLOPT_FOUND)
        message(STATUS "Found NLopt: ${NLOPT_INCLUDE_DIRS}")
        include_directories(${NLOPT_INCLUDE_DIRS})
        link_directories(${NLOPT_LIBRARY_DIRS})
    else()
        # 尝试直接查找
        find_library(NLOPT_LIBRARIES nlopt)
        find_path(NLOPT_INCLUDE_DIRS nlopt.h)
        if(NLOPT_LIBRARIES AND NLOPT_INCLUDE_DIRS)
            set(NLOPT_FOUND TRUE)
            message(STATUS "Found NLopt: ${NLOPT_INCLUDE_DIRS}")
            include_directories(${NLOPT_INCLUDE_DIRS})
        endif()
    endif()
endif()

# 查找OSQP (libmpc++依赖)
set(OSQP_FOUND FALSE)
if(PkgConfig_FOUND)
    pkg_check_modules(OSQP QUIET osqp)
    if(OSQP_FOUND)
        message(STATUS "Found OSQP: ${OSQP_INCLUDE_DIRS}")
        include_directories(${OSQP_INCLUDE_DIRS})
        link_directories(${OSQP_LIBRARY_DIRS})
    else()
        # 尝试直接查找
        find_library(OSQP_LIBRARIES osqp)
        find_path(OSQP_INCLUDE_DIRS osqp.h)
        if(OSQP_LIBRARIES AND OSQP_INCLUDE_DIRS)
            set(OSQP_FOUND TRUE)
            message(STATUS "Found OSQP: ${OSQP_INCLUDE_DIRS}")
            include_directories(${OSQP_INCLUDE_DIRS})
        endif()
    endif()
endif()

# 设置编译定义
add_definitions(-DWITH_LIBMPC)
message(STATUS "Building with embedded libmpc++")

# 如果找不到NLopt或OSQP，给出警告但继续编译
if(NOT NLOPT_FOUND)
    message(WARNING "NLopt not found. Some libmpc++ features may not work.")
endif()

if(NOT OSQP_FOUND)
    message(WARNING "OSQP not found. Some libmpc++ features may not work.")
endif()

# 收集源文件
file(GLOB_RECURSE SOURCES 
    "src/*.cpp"
    "src/*/*.cpp"
)

# 收集头文件
file(GLOB_RECURSE HEADERS 
    "include/*.hpp"
    "include/*/*.hpp"
    "third_party/mpc/*.hpp"
)

# 创建库
add_library(vcu_core ${SOURCES})
target_include_directories(vcu_core PUBLIC 
    include 
    third_party
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(vcu_core Eigen3::Eigen)

# 链接libmpc++的依赖
if(NLOPT_FOUND)
    target_link_libraries(vcu_core ${NLOPT_LIBRARIES})
endif()

if(OSQP_FOUND)
    target_link_libraries(vcu_core ${OSQP_LIBRARIES})
endif()

# 创建测试可执行文件
add_executable(system_test tests/system_test.cpp)
target_include_directories(system_test PRIVATE 
    include 
    third_party
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(system_test vcu_core Eigen3::Eigen)

# 链接libmpc++的依赖
if(NLOPT_FOUND)
    target_link_libraries(system_test ${NLOPT_LIBRARIES})
endif()

if(OSQP_FOUND)
    target_link_libraries(system_test ${OSQP_LIBRARIES})
endif()

# 设置输出目录
set_target_properties(system_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# 安装规则
install(TARGETS vcu_core system_test
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY third_party/ DESTINATION include)

# 打印配置信息
message(STATUS "=== Build Configuration ===")
message(STATUS "CMAKE_CXX_COMPILER: ${CMAKE_CXX_COMPILER}")
message(STATUS "CMAKE_CXX_STANDARD: ${CMAKE_CXX_STANDARD}")
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "WITH_LIBMPC: ON (embedded)")
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "NLOPT_FOUND: ${NLOPT_FOUND}")
if(NLOPT_FOUND)
    message(STATUS "NLOPT_INCLUDE_DIRS: ${NLOPT_INCLUDE_DIRS}")
    message(STATUS "NLOPT_LIBRARIES: ${NLOPT_LIBRARIES}")
endif()
message(STATUS "OSQP_FOUND: ${OSQP_FOUND}")
if(OSQP_FOUND)
    message(STATUS "OSQP_INCLUDE_DIRS: ${OSQP_INCLUDE_DIRS}")
    message(STATUS "OSQP_LIBRARIES: ${OSQP_LIBRARIES}")
endif()
message(STATUS "===========================")

# 添加编译测试
add_custom_target(test_compile
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target system_test
    COMMENT "Testing compilation of system_test"
)
