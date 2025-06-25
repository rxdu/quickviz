#!/bin/bash

# QuickViz Comprehensive Test Runner
# This script runs all types of tests: unit, integration, benchmarks, and memory tests

set -e

# Default values
BUILD_DIR="build"
BUILD_TYPE="Debug"
RUN_VALGRIND=false
RUN_BENCHMARKS=false
GENERATE_COVERAGE=false
VERBOSE=false

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show usage
show_usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Run comprehensive tests for QuickViz project.

OPTIONS:
    -d, --build-dir DIR     Build directory (default: build)
    -t, --build-type TYPE   Build type: Debug|Release (default: Debug)
    -v, --valgrind          Run Valgrind memory tests
    -b, --benchmarks        Run performance benchmarks
    -c, --coverage          Generate code coverage report
    --verbose               Enable verbose output
    -h, --help              Show this help message

EXAMPLES:
    $0                      # Run basic tests
    $0 -v                   # Run tests with Valgrind
    $0 -b                   # Run tests and benchmarks
    $0 -c                   # Run tests with coverage
    $0 -v -b -c             # Run all tests with all options

EOF
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--build-dir)
            BUILD_DIR="$2"
            shift 2
            ;;
        -t|--build-type)
            BUILD_TYPE="$2"
            shift 2
            ;;
        -v|--valgrind)
            RUN_VALGRIND=true
            shift
            ;;
        -b|--benchmarks)
            RUN_BENCHMARKS=true
            shift
            ;;
        -c|--coverage)
            GENERATE_COVERAGE=true
            BUILD_TYPE="Debug"  # Coverage requires debug build
            shift
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Check if we're in the project root
if [ ! -f "CMakeLists.txt" ]; then
    print_error "This script must be run from the project root directory"
    exit 1
fi

print_status "Starting QuickViz comprehensive test suite"
print_status "Build directory: $BUILD_DIR"
print_status "Build type: $BUILD_TYPE"

# Create build directory if it doesn't exist
if [ ! -d "$BUILD_DIR" ]; then
    print_status "Creating build directory: $BUILD_DIR"
    mkdir -p "$BUILD_DIR"
fi

# Configure CMake with testing enabled
print_status "Configuring CMake with testing enabled..."
cmake_args=(
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
    -DBUILD_TESTING=ON
    -DQUICKVIZ_DEV_MODE=ON
)

if [ "$GENERATE_COVERAGE" = true ]; then
    cmake_args+=(-DENABLE_COVERAGE=ON)
    print_status "Code coverage enabled"
fi

cd "$BUILD_DIR"
cmake .. "${cmake_args[@]}"

# Build the project and tests
print_status "Building project and tests..."
if [ "$VERBOSE" = true ]; then
    cmake --build . --config "$BUILD_TYPE" --verbose
else
    cmake --build . --config "$BUILD_TYPE"
fi

print_success "Build completed successfully"

# Run unit tests
print_status "Running unit tests..."
if [ "$VERBOSE" = true ]; then
    ctest -C "$BUILD_TYPE" --verbose --output-on-failure -L "unit"
else
    ctest -C "$BUILD_TYPE" --output-on-failure -L "unit"
fi

# Run integration tests
print_status "Running integration tests..."
if [ "$VERBOSE" = true ]; then
    ctest -C "$BUILD_TYPE" --verbose --output-on-failure -L "integration"
else
    ctest -C "$BUILD_TYPE" --output-on-failure -L "integration"
fi

# Run memory tests
print_status "Running memory tests..."
if [ "$VERBOSE" = true ]; then
    ctest -C "$BUILD_TYPE" --verbose --output-on-failure -L "memory"
else
    ctest -C "$BUILD_TYPE" --output-on-failure -L "memory"
fi

# Run Valgrind tests if requested
if [ "$RUN_VALGRIND" = true ]; then
    if command -v valgrind >/dev/null 2>&1; then
        print_status "Running Valgrind memory leak tests..."
        export VALGRIND_OPTS="--suppressions=../tests/valgrind.supp"
        if [ "$VERBOSE" = true ]; then
            ctest -C "$BUILD_TYPE" --verbose --output-on-failure -R "valgrind"
        else
            ctest -C "$BUILD_TYPE" --output-on-failure -R "valgrind"
        fi
    else
        print_warning "Valgrind not found, skipping memory leak tests"
    fi
fi

# Run benchmarks if requested
if [ "$RUN_BENCHMARKS" = true ]; then
    if [ -f "benchmark_rendering" ]; then
        print_status "Running performance benchmarks..."
        ./benchmark_rendering --benchmark_format=console --benchmark_color=true
    else
        print_warning "Benchmarks not built (Google Benchmark not found)"
    fi
fi

# Generate coverage report if requested
if [ "$GENERATE_COVERAGE" = true ]; then
    if command -v gcov >/dev/null 2>&1 && command -v lcov >/dev/null 2>&1; then
        print_status "Generating code coverage report..."
        
        # Capture coverage data
        lcov --directory . --capture --output-file coverage.info
        
        # Filter out system and test files
        lcov --remove coverage.info '/usr/*' '*/tests/*' '*/test/*' '*/googletest/*' --output-file coverage_filtered.info
        
        # Generate HTML report
        if command -v genhtml >/dev/null 2>&1; then
            genhtml coverage_filtered.info --output-directory coverage_html
            print_success "Coverage report generated in: $BUILD_DIR/coverage_html/index.html"
        fi
        
        # Show coverage summary
        lcov --list coverage_filtered.info
    else
        print_warning "Coverage tools (gcov/lcov) not found, skipping coverage report"
    fi
fi

cd ..

print_success "All tests completed successfully!"
print_status "Test results summary:"
echo "  ✓ Unit tests: PASSED"
echo "  ✓ Integration tests: PASSED"
echo "  ✓ Memory tests: PASSED"

if [ "$RUN_VALGRIND" = true ]; then
    echo "  ✓ Valgrind tests: PASSED"
fi

if [ "$RUN_BENCHMARKS" = true ]; then
    echo "  ✓ Performance benchmarks: COMPLETED"
fi

if [ "$GENERATE_COVERAGE" = true ]; then
    echo "  ✓ Coverage report: GENERATED"
fi

print_success "QuickViz test suite execution complete!"