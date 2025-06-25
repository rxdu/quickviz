# QuickViz Testing Framework

This directory contains a comprehensive testing suite for the QuickViz visualization library, including unit tests, integration tests, performance benchmarks, and memory leak detection.

## Test Structure

```
tests/
├── unit/                   # Unit tests for individual components
├── integration/           # Integration tests for component interactions
├── benchmarks/           # Performance benchmarks
├── memory/              # Memory leak detection tests
├── data/               # Test data and fixtures (if any)
├── valgrind.supp       # Valgrind suppression file
└── CMakeLists.txt      # Test build configuration
```

## Test Categories

### 1. Unit Tests (`unit/`)

- **Event System Tests** (`test_event_system.cpp`): Tests for event dispatcher and async event handling
- **Buffer Registry Tests** (`test_buffer_registry.cpp`): Tests for buffer management and registry
- Additional unit tests for core components

### 2. Integration Tests (`integration/`)

- **Renderer Pipeline Tests** (`test_renderer_pipeline.cpp`): End-to-end rendering pipeline testing
- **ImView Integration Tests** (`test_imview_integration.cpp`): GUI component integration testing

### 3. Performance Benchmarks (`benchmarks/`)

- **Rendering Benchmarks** (`benchmark_rendering.cpp`): Performance tests for rendering operations
- Point cloud rendering performance
- Multiple object rendering tests
- Scene management benchmarks

### 4. Memory Tests (`memory/`)

- **Memory Leak Tests** (`test_memory_leaks.cpp`): OpenGL resource cleanup verification
- Valgrind integration for automated memory leak detection

## Running Tests

### Quick Start

```bash
# From project root
mkdir build && cd build
cmake .. -DBUILD_TESTING=ON
cmake --build .
ctest --output-on-failure
```

### Using the Test Runner Script

The comprehensive test runner script provides advanced testing options:

```bash
# Basic test run
./scripts/run_tests.sh

# Run with Valgrind memory checking
./scripts/run_tests.sh --valgrind

# Run with performance benchmarks
./scripts/run_tests.sh --benchmarks

# Generate code coverage report
./scripts/run_tests.sh --coverage

# Run all tests with all options
./scripts/run_tests.sh --valgrind --benchmarks --coverage --verbose
```

### Test Runner Options

- `-d, --build-dir DIR`: Specify build directory (default: build)
- `-t, --build-type TYPE`: Build type Debug|Release (default: Debug)
- `-v, --valgrind`: Run Valgrind memory tests
- `-b, --benchmarks`: Run performance benchmarks
- `-c, --coverage`: Generate code coverage report
- `--verbose`: Enable verbose output
- `-h, --help`: Show help message

## Test Configuration

### CMake Options

- `BUILD_TESTING`: Enable/disable test building (default: OFF)
- `QUICKVIZ_DEV_MODE`: Development mode (forces test building)
- `ENABLE_COVERAGE`: Enable code coverage (requires Debug build)

### Dependencies

The testing framework requires:

**Required:**
- Google Test (GTest) - included as submodule
- OpenGL libraries for graphics tests

**Optional:**
- Google Benchmark - for performance tests
- Valgrind - for memory leak detection
- lcov/gcov - for code coverage reports

## CI/CD Integration

Tests are automatically run in GitHub Actions on:
- Ubuntu 22.04 and 24.04
- Both full and minimal builds
- Pull requests and main branch pushes

The CI pipeline:
1. Builds the project with testing enabled
2. Runs all test categories
3. Uploads test results as artifacts
4. Uses Valgrind suppression for known false positives

## Writing New Tests

### Unit Tests

```cpp
#include <gtest/gtest.h>
#include "your_component.hpp"

class YourComponentTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup code
    }
    
    void TearDown() override {
        // Cleanup code
    }
};

TEST_F(YourComponentTest, TestSomething) {
    EXPECT_EQ(expected, actual);
}
```

### Integration Tests

Focus on component interactions and end-to-end workflows:

```cpp
TEST_F(IntegrationTest, RendererPipelineWorkflow) {
    // Create viewer and scene
    auto viewer = std::make_unique<Viewer>("Test", 800, 600);
    auto scene = std::make_shared<GlSceneManager>("TestScene");
    
    // Add objects and test rendering
    viewer->AddSceneObject(scene);
    // ... test logic
}
```

### Performance Benchmarks

```cpp
#include <benchmark/benchmark.h>

static void BM_YourFunction(benchmark::State& state) {
    for (auto _ : state) {
        // Code to benchmark
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_YourFunction);
```

## Memory Testing

### Valgrind Integration

The test suite includes Valgrind integration for memory leak detection:

```bash
# Run specific test with Valgrind
valgrind --tool=memcheck --leak-check=full \
         --suppressions=tests/valgrind.supp \
         ./build/test_memory_leaks
```

### Suppression File

The `valgrind.supp` file contains suppressions for known false positives from:
- OpenGL drivers (Mesa, NVIDIA)
- X11/Wayland libraries
- System libraries
- Third-party dependencies

## Test Data

Test fixtures and sample data should be placed in the `data/` directory and automatically copied to the build directory.

## Best Practices

1. **Test Isolation**: Each test should be independent and not rely on other tests
2. **Resource Cleanup**: Always clean up OpenGL resources in tests
3. **Headless Testing**: Use headless mode for graphics tests in CI
4. **Error Handling**: Test both success and error conditions
5. **Performance Awareness**: Keep test execution time reasonable
6. **Clear Naming**: Use descriptive test names that explain what is being tested

## Troubleshooting

### Common Issues

1. **OpenGL Context Errors**: Ensure tests create proper OpenGL context
2. **Memory Leaks in CI**: Check Valgrind suppressions for false positives
3. **Benchmark Variability**: Run benchmarks multiple times for stability
4. **Test Timeouts**: Increase timeout for longer integration tests

### Debug Mode

For debugging test failures:

```bash
# Run specific test with debug output
ctest -R "test_name" --verbose --output-on-failure

# Run under debugger
gdb ./build/test_executable
```

## Contributing

When adding new features to QuickViz:

1. Add corresponding unit tests for new components
2. Update integration tests if changing interfaces
3. Add performance benchmarks for performance-critical code
4. Test memory management for OpenGL resources
5. Update this documentation for new test categories

## Coverage Goals

Target coverage levels:
- Unit Tests: > 80% line coverage
- Integration Tests: All major workflows covered
- Performance Tests: All performance-critical paths benchmarked
- Memory Tests: All OpenGL resource lifecycles tested