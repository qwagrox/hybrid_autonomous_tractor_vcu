#!/bin/bash
# scripts/build.sh

set -e

echo "Building Autonomous Tractor VCU System..."
echo "=========================================="

# 创建构建目录
mkdir -p build
cd build

# 检查构建类型
BUILD_TYPE="RelWithDebInfo"
if [ "$1" == "debug" ]; then
    BUILD_TYPE="Debug"
    echo "Building in Debug mode..."
elif [ "$1" == "release" ]; then
    BUILD_TYPE="Release"
    echo "Building in Release mode..."
else
    echo "Building in RelWithDebInfo mode..."
fi

# 运行CMake
cmake -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ..
if [ $? -ne 0 ]; then
    echo "CMake configuration failed!"
    exit 1
fi

# 获取CPU核心数
CORES=$(nproc)
echo "Building with ${CORES} cores..."

# 编译
make -j${CORES}
if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

# 运行测试
echo "Running tests..."
ctest --output-on-failure
if [ $? -ne 0 ]; then
    echo "Tests failed!"
    exit 1
fi

echo "Build completed successfully!"
echo "Binary: build/vcu_system"