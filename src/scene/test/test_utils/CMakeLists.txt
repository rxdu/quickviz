# Test-only utilities for scene tests. Not part of the public library;
# never link these from non-test code.
#
# Provides simple renderables and helpers used across the renderable +
# selection + integration tests. Triangle lives here (rather than in the
# public scene library) because every real use is a test fixture or
# tutorial example — not application code.

add_library(scene_test_utils STATIC
    triangle.cpp)

target_link_libraries(scene_test_utils PUBLIC scene)

target_include_directories(scene_test_utils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR})
