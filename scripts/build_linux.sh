#!/bin/bash

# Build script for Linux platform
# Copyright 2025 Manus AI

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_ROOT/build_linux"

echo "=== Building VCU Controller for Linux Platform ==="
echo "Project Root: $PROJECT_ROOT"
echo "Build Directory: $BUILD_DIR"

# Create build directory
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure with CMake
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DTARGET_PLATFORM=LINUX \
    -DCMAKE_INSTALL_PREFIX="$PROJECT_ROOT/install_linux" \
    "$PROJECT_ROOT"

# Build
make -j$(nproc)

echo "=== Linux Build Complete ==="
echo "Binaries available in: $BUILD_DIR"
echo "To install, run: make install"
echo "To run tests, run: make test"
