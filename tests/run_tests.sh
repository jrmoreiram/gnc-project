#!/bin/bash

# GNC Project Test Runner Script
# This script builds and runs all tests

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"
TESTS_DIR="$PROJECT_ROOT/tests"

echo "🧪 GNC Project Test Suite"
echo "=========================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if build directory exists
if [ ! -d "$BUILD_DIR" ]; then
    echo -e "${YELLOW}Building project...${NC}"
    cd "$PROJECT_ROOT"
    cmake -S . -B build
    cmake --build build -j
    echo -e "${GREEN}Build completed!${NC}"
    echo ""
fi

# Check if tests executable exists
if [ ! -f "$BUILD_DIR/tests/gnc_tests" ]; then
    echo -e "${YELLOW}Building tests...${NC}"
    cd "$PROJECT_ROOT"
    cmake -S . -B build
    cmake --build build -j
    
    if [ ! -f "$BUILD_DIR/tests/gnc_tests" ]; then
        echo -e "${RED}Error: Tests executable not found!${NC}"
        echo "Make sure CMakeLists.txt includes tests subdirectory"
        exit 1
    fi
    echo -e "${GREEN}Tests built successfully!${NC}"
    echo ""
fi

# Run the tests
echo -e "${BLUE}Running tests...${NC}"
echo ""

cd "$BUILD_DIR"
ctest --output-on-failure -V

# Check test results
if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}✓ All tests passed!${NC}"
    exit 0
else
    echo ""
    echo -e "${RED}✗ Some tests failed!${NC}"
    exit 1
fi
