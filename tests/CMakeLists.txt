# CMakeLists.txt for QuickViz Tests
# Comprehensive test suite including unit tests, integration tests, benchmarks, and memory tests

cmake_minimum_required(VERSION 3.15)

# Test configuration
if(NOT BUILD_TESTING)
    return()
endif()

# Find required packages
find_package(GTest REQUIRED)

# Optional: Find Google Benchmark for performance tests
find_package(benchmark QUIET)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/src)

# Test utilities library
add_library(test_utils INTERFACE)
target_include_directories(test_utils INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/utils)

# ==============================================================================
# Unit Tests
# ==============================================================================

# Event system unit tests
add_executable(test_event_system
    unit/test_event_system.cpp
)
target_link_libraries(test_event_system
    PRIVATE
    GTest::gtest_main
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
    GTest::gtest_main
    core
    test_utils
)
gtest_discover_tests(test_buffer_registry
    PROPERTIES LABELS "unit"
)

# ==============================================================================
# Integration Tests
# ==============================================================================

# Renderer pipeline integration tests
add_executable(test_renderer_pipeline
    integration/test_renderer_pipeline.cpp
)
target_link_libraries(test_renderer_pipeline
    PRIVATE
    GTest::gtest_main
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
    GTest::gtest_main
    imview
    test_utils
)
gtest_discover_tests(test_imview_integration
    PROPERTIES LABELS "integration"
)

# ==============================================================================
# Memory Leak Tests
# ==============================================================================

# Memory leak detection tests
add_executable(test_memory_leaks
    memory/test_memory_leaks.cpp
)
target_link_libraries(test_memory_leaks
    PRIVATE
    GTest::gtest_main
    imview
    renderer
    test_utils
)
gtest_discover_tests(test_memory_leaks
    PROPERTIES LABELS "memory"
)

# ==============================================================================
# Performance Benchmarks (Optional)
# ==============================================================================

if(benchmark_FOUND)
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
    message(STATUS "Google Benchmark not found - skipping performance benchmarks")
endif()

# ==============================================================================
# Valgrind Memory Testing
# ==============================================================================

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
message(STATUS "  Unit Tests: Enabled")
message(STATUS "  Integration Tests: Enabled") 
message(STATUS "  Memory Tests: Enabled")
if(benchmark_FOUND)
    message(STATUS "  Performance Benchmarks: Enabled")
else()
    message(STATUS "  Performance Benchmarks: Disabled (Google Benchmark not found)")
endif()
if(VALGRIND_EXECUTABLE)
    message(STATUS "  Valgrind Memory Testing: Enabled")
else()
    message(STATUS "  Valgrind Memory Testing: Disabled (Valgrind not found)")
endif()
if(ENABLE_COVERAGE)
    message(STATUS "  Code Coverage: Enabled")
else()
    message(STATUS "  Code Coverage: Disabled")
endif()