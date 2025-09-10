# SceneManagerBridge Test Application
# Interactive manual test for demonstrating bridge integration

add_executable(object_management
    main.cpp
    bridge_scene_manager.cpp
    bridge_control_panel.cpp
)

target_link_libraries(object_management
    scenegraph     # SceneManagerBridge
    gldraw         # OpenGL rendering
    imview         # UI framework
    core           # Utilities
    ${OPENGL_LIBRARIES}
    glfw
)

target_include_directories(object_management PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Install the executable
install(TARGETS object_management
    RUNTIME DESTINATION bin
    COMPONENT applications
)