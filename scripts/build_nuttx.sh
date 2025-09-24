#!/bin/bash

# Build script for NuttX platform
# Copyright 2025 Manus AI

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_ROOT/build_nuttx"

echo "=== Building VCU Controller for NuttX Platform ==="
echo "Project Root: $PROJECT_ROOT"
echo "Build Directory: $BUILD_DIR"

# Check for NuttX toolchain
if [ -z "$NUTTX_TOOLCHAIN_PATH" ]; then
    echo "Warning: NUTTX_TOOLCHAIN_PATH not set. Using default cross-compiler."
    CROSS_COMPILE="arm-none-eabi-"
else
    CROSS_COMPILE="$NUTTX_TOOLCHAIN_PATH/arm-none-eabi-"
fi

# Create build directory
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure with CMake for cross-compilation
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DTARGET_PLATFORM=NUTTX \
    -DCMAKE_SYSTEM_NAME=Generic \
    -DCMAKE_SYSTEM_PROCESSOR=arm \
    -DCMAKE_C_COMPILER="${CROSS_COMPILE}gcc" \
    -DCMAKE_CXX_COMPILER="${CROSS_COMPILE}g++" \
    -DCMAKE_ASM_COMPILER="${CROSS_COMPILE}gcc" \
    -DCMAKE_OBJCOPY="${CROSS_COMPILE}objcopy" \
    -DCMAKE_OBJDUMP="${CROSS_COMPILE}objdump" \
    -DCMAKE_SIZE="${CROSS_COMPILE}size" \
    -DCMAKE_DEBUGGER="${CROSS_COMPILE}gdb" \
    -DCMAKE_CPPFILT="${CROSS_COMPILE}c++filt" \
    -DCMAKE_INSTALL_PREFIX="$PROJECT_ROOT/install_nuttx" \
    "$PROJECT_ROOT"

# Build
make -j$(nproc)

echo "=== NuttX Build Complete ==="
echo "Binaries available in: $BUILD_DIR"
echo "Note: Testing is not available for NuttX builds"
echo "To install, run: make install"
