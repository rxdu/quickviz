# vscene module CMakeLists.txt  
# Virtual Scene Layer - High-level interactive 3D scene management

# Create vscene library with concrete implementation
add_library(vscene
    src/virtual_object.cpp
    src/virtual_sphere.cpp
    src/virtual_scene.cpp
    src/gl_render_backend.cpp
)

target_include_directories(vscene PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies  
target_link_libraries(vscene PUBLIC
    core           # Event system base
    imview         # Panel integration  
    gldraw         # Render backend
)

# Test executable to demonstrate workflow (won't compile yet - that's expected)
if(BUILD_TESTING)
    add_subdirectory(test)
endif()

# Install headers
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Export target
install(TARGETS vscene
    EXPORT quickvizTargets
)

# Interface validation notes:
# - All headers are designed as interface specifications
# - Implementation can be added incrementally  
# - Test demonstrates intended usage workflow
# - Focus is on API design and integration patterns