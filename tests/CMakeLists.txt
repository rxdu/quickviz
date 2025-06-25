# CMakeLists.txt for QuickViz Tests
# Comprehensive test suite including unit tests, integration tests, benchmarks, and memory tests

cmake_minimum_required(VERSION 3.15)

# Test configuration
if(NOT BUILD_TESTING)
    return()
endif()

# Check if googletest is available (from third_party submodule)
if(NOT TARGET gtest_main)
    message(FATAL_ERROR "GoogleTest targets not found. Make sure googletest submodule is initialized and third_party is added before tests.")
endif()

# Detect CI environment
if(DEFINED ENV{CI} OR DEFINED ENV{GITHUB_ACTIONS})
    set(RUNNING_IN_CI TRUE)
    message(STATUS "Detected CI environment - will skip OpenGL/GUI tests")
else()
    set(RUNNING_IN_CI FALSE)
    message(STATUS "Not in CI environment - will run all tests")
endif()

# Optional: Find Google Benchmark for performance tests
find_package(benchmark QUIET)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/src)

# Test utilities library
add_library(test_utils INTERFACE)
target_include_directories(test_utils INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/utils)

# ==============================================================================
# Unit Tests (Always run)
# ==============================================================================

# Event system unit tests
add_executable(test_event_system
    unit/test_event_system.cpp
)
target_link_libraries(test_event_system
    PRIVATE
    gtest_main
    core
    test_utils
)
gtest_discover_tests(test_event_system
    PROPERTIES LABELS "unit"
)

# Buffer registry unit tests
add_executable(test_buffer_registry
    unit/test_buffer_registry.cpp
)
target_link_libraries(test_buffer_registry
    PRIVATE
    gtest_main
    core
    test_utils
)
gtest_discover_tests(test_buffer_registry
    PROPERTIES LABELS "unit"
)

# ==============================================================================
# Integration Tests (Skip in CI)
# ==============================================================================

if(NOT RUNNING_IN_CI)
    # Renderer pipeline integration tests
    add_executable(test_renderer_pipeline
        integration/test_renderer_pipeline.cpp
    )
    target_link_libraries(test_renderer_pipeline
        PRIVATE
        gtest_main
        imview
        renderer
        test_utils
    )
    gtest_discover_tests(test_renderer_pipeline
        PROPERTIES LABELS "integration"
    )

    # ImView integration tests
    add_executable(test_imview_integration
        integration/test_imview_integration.cpp
    )
    target_link_libraries(test_imview_integration
        PRIVATE
        gtest_main
        imview
        test_utils
    )
    gtest_discover_tests(test_imview_integration
        PROPERTIES LABELS "integration"
    )
else()
    message(STATUS "Skipping OpenGL/GUI integration tests in CI environment")
endif()

# ==============================================================================
# Memory Leak Tests (Skip in CI)
# ==============================================================================

if(NOT RUNNING_IN_CI)
    # Memory leak detection tests
    add_executable(test_memory_leaks
        memory/test_memory_leaks.cpp
    )
    target_link_libraries(test_memory_leaks
        PRIVATE
        gtest_main
        imview
        renderer
        test_utils
    )
    gtest_discover_tests(test_memory_leaks
        PROPERTIES LABELS "memory"
    )
else()
    message(STATUS "Skipping OpenGL memory leak tests in CI environment")
endif()

# ==============================================================================
# Performance Benchmarks (Optional)
# ==============================================================================

if(benchmark_FOUND)
    if(NOT RUNNING_IN_CI)
        # Rendering performance benchmarks
        add_executable(benchmark_rendering
            benchmarks/benchmark_rendering.cpp
        )
        target_link_libraries(benchmark_rendering
            PRIVATE
            benchmark::benchmark
            imview
            renderer
            core
        )
        
        # Add benchmark as a test (optional)
        add_test(NAME benchmark_rendering_test
            COMMAND benchmark_rendering --benchmark_min_time=0.1
        )
        
        message(STATUS "Google Benchmark found - building performance benchmarks")
    else()
        message(STATUS "Google Benchmark found but skipping OpenGL benchmarks in CI environment")
    endif()
else()
    message(STATUS "Google Benchmark not found - skipping performance benchmarks")
endif()

# ==============================================================================
# Valgrind Memory Testing (Skip in CI)
# ==============================================================================

if(NOT RUNNING_IN_CI)
    # Find Valgrind for memory testing
    find_program(VALGRIND_EXECUTABLE valgrind)

    if(VALGRIND_EXECUTABLE)
        # Add Valgrind tests for memory leak detection
        add_test(NAME memory_leak_valgrind_test
            COMMAND ${VALGRIND_EXECUTABLE}
            --tool=memcheck
            --leak-check=full
            --show-leak-kinds=all
            --track-origins=yes
            --verbose
            --error-exitcode=1
            $<TARGET_FILE:test_memory_leaks>
        )
        
        # Add Valgrind test for renderer pipeline
        add_test(NAME renderer_valgrind_test
            COMMAND ${VALGRIND_EXECUTABLE}
            --tool=memcheck
            --leak-check=full
            --show-leak-kinds=all
            --error-exitcode=1
            $<TARGET_FILE:test_renderer_pipeline>
        )
        
        message(STATUS "Valgrind found - adding memory leak tests")
    else()
        message(STATUS "Valgrind not found - skipping memory leak tests")
    endif()
else()
    message(STATUS "Skipping Valgrind tests in CI environment")
endif()

# ==============================================================================
# Test Data and Fixtures
# ==============================================================================

# Create test data directory
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test_data)

# Copy test resources if they exist
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/data)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data/
         DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/test_data)
endif()

# ==============================================================================
# Coverage Testing (Optional)
# ==============================================================================

option(ENABLE_COVERAGE "Enable code coverage testing" OFF)

if(ENABLE_COVERAGE AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    if(CMAKE_COMPILER_IS_GNUCXX)
        target_compile_options(test_event_system PRIVATE --coverage)
        target_link_options(test_event_system PRIVATE --coverage)
        
        target_compile_options(test_buffer_registry PRIVATE --coverage)
        target_link_options(test_buffer_registry PRIVATE --coverage)
        
        target_compile_options(test_renderer_pipeline PRIVATE --coverage)
        target_link_options(test_renderer_pipeline PRIVATE --coverage)
        
        target_compile_options(test_imview_integration PRIVATE --coverage)
        target_link_options(test_imview_integration PRIVATE --coverage)
        
        target_compile_options(test_memory_leaks PRIVATE --coverage)
        target_link_options(test_memory_leaks PRIVATE --coverage)
        
        message(STATUS "Code coverage enabled")
    endif()
endif()

# ==============================================================================
# Test Configuration Summary
# ==============================================================================

message(STATUS "Test Configuration Summary:")
message(STATUS "  CI Environment: ${RUNNING_IN_CI}")
message(STATUS "  Unit Tests: Enabled")
if(RUNNING_IN_CI)
    message(STATUS "  Integration Tests: Disabled (CI environment)")
    message(STATUS "  Memory Tests: Disabled (CI environment)")
    message(STATUS "  Valgrind Memory Testing: Disabled (CI environment)")
else()
    message(STATUS "  Integration Tests: Enabled")
    message(STATUS "  Memory Tests: Enabled")
    if(VALGRIND_EXECUTABLE)
        message(STATUS "  Valgrind Memory Testing: Enabled")
    else()
        message(STATUS "  Valgrind Memory Testing: Disabled (Valgrind not found)")
    endif()
endif()
if(benchmark_FOUND)
    if(RUNNING_IN_CI)
        message(STATUS "  Performance Benchmarks: Disabled (CI environment)")
    else()
        message(STATUS "  Performance Benchmarks: Enabled")
    endif()
else()
    message(STATUS "  Performance Benchmarks: Disabled (Google Benchmark not found)")
endif()
if(ENABLE_COVERAGE)
    message(STATUS "  Code Coverage: Enabled")
else()
    message(STATUS "  Code Coverage: Disabled")
endif()