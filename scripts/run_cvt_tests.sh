#!/bin/bash

# CVT Strategy Factory Test Runner Script
# This script builds and runs all CVT-related tests

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}CVT Strategy Factory Test Runner${NC}"
echo "=================================="

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ]; then
    echo -e "${RED}Error: Please run this script from the VCU controller root directory${NC}"
    exit 1
fi

# Create build directory
BUILD_DIR="build_test"
if [ ! -d "$BUILD_DIR" ]; then
    echo -e "${YELLOW}Creating build directory...${NC}"
    mkdir -p "$BUILD_DIR"
fi

cd "$BUILD_DIR"

# Configure CMake with testing enabled
echo -e "${YELLOW}Configuring CMake...${NC}"
cmake .. -DCMAKE_BUILD_TYPE=Debug -DENABLE_TESTING=ON

# Build the project
echo -e "${YELLOW}Building project...${NC}"
make -j$(nproc)

# Check if test executables exist
TEST_EXECUTABLES=(
    "tests/cvt/test_cvt_strategy_factory"
    "tests/cvt/test_hmcvt_vendor1_strategy"
    "tests/cvt/test_cvt_config"
    "tests/integration/test_cvt_integration"
)

echo -e "${YELLOW}Checking test executables...${NC}"
for test_exe in "${TEST_EXECUTABLES[@]}"; do
    if [ ! -f "$test_exe" ]; then
        echo -e "${RED}Warning: Test executable $test_exe not found${NC}"
    else
        echo -e "${GREEN}Found: $test_exe${NC}"
    fi
done

# Run CVT Strategy Factory tests
echo -e "${YELLOW}Running CVT Strategy Factory tests...${NC}"
if [ -f "tests/cvt/test_cvt_strategy_factory" ]; then
    ./tests/cvt/test_cvt_strategy_factory --gtest_output=xml:test_results_factory.xml
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}CVT Strategy Factory tests: PASSED${NC}"
    else
        echo -e "${RED}CVT Strategy Factory tests: FAILED${NC}"
    fi
else
    echo -e "${YELLOW}Skipping CVT Strategy Factory tests (executable not found)${NC}"
fi

# Run HMCVT_Vendor1 Strategy tests
echo -e "${YELLOW}Running HMCVT_Vendor1 Strategy tests...${NC}"
if [ -f "tests/cvt/test_hmcvt_vendor1_strategy" ]; then
    ./tests/cvt/test_hmcvt_vendor1_strategy --gtest_output=xml:test_results_hmcvt.xml
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}HMCVT_Vendor1 Strategy tests: PASSED${NC}"
    else
        echo -e "${RED}HMCVT_Vendor1 Strategy tests: FAILED${NC}"
    fi
else
    echo -e "${YELLOW}Skipping HMCVT_Vendor1 Strategy tests (executable not found)${NC}"
fi

# Run CVT Config tests
echo -e "${YELLOW}Running CVT Config tests...${NC}"
if [ -f "tests/cvt/test_cvt_config" ]; then
    ./tests/cvt/test_cvt_config --gtest_output=xml:test_results_config.xml
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}CVT Config tests: PASSED${NC}"
    else
        echo -e "${RED}CVT Config tests: FAILED${NC}"
    fi
else
    echo -e "${YELLOW}Skipping CVT Config tests (executable not found)${NC}"
fi

# Run Integration tests
echo -e "${YELLOW}Running CVT Integration tests...${NC}"
if [ -f "tests/integration/test_cvt_integration" ]; then
    ./tests/integration/test_cvt_integration --gtest_output=xml:test_results_integration.xml
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}CVT Integration tests: PASSED${NC}"
    else
        echo -e "${RED}CVT Integration tests: FAILED${NC}"
    fi
else
    echo -e "${YELLOW}Skipping CVT Integration tests (executable not found)${NC}"
fi

# Run all tests using CTest if available
echo -e "${YELLOW}Running all tests with CTest...${NC}"
if command -v ctest &> /dev/null; then
    ctest --output-on-failure --verbose
else
    echo -e "${YELLOW}CTest not available, skipping comprehensive test run${NC}"
fi

# Generate test report
echo -e "${YELLOW}Generating test report...${NC}"
if command -v find &> /dev/null; then
    XML_FILES=$(find . -name "test_results_*.xml" 2>/dev/null)
    if [ ! -z "$XML_FILES" ]; then
        echo -e "${GREEN}Test result XML files generated:${NC}"
        for xml_file in $XML_FILES; do
            echo "  - $xml_file"
        done
    fi
fi

# Check for memory leaks if valgrind is available
if command -v valgrind &> /dev/null; then
    echo -e "${YELLOW}Running memory leak check on CVT Strategy Factory test...${NC}"
    if [ -f "tests/cvt/test_cvt_strategy_factory" ]; then
        valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes \
                 --xml=yes --xml-file=valgrind_factory.xml \
                 ./tests/cvt/test_cvt_strategy_factory > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}Memory leak check: PASSED${NC}"
        else
            echo -e "${YELLOW}Memory leak check: Check valgrind_factory.xml for details${NC}"
        fi
    fi
else
    echo -e "${YELLOW}Valgrind not available, skipping memory leak check${NC}"
fi

# Summary
echo ""
echo -e "${YELLOW}Test Summary${NC}"
echo "============"
echo "Build directory: $BUILD_DIR"
echo "Test results available in XML format"
echo ""

# Return to original directory
cd ..

echo -e "${GREEN}CVT Strategy Factory testing completed!${NC}"
