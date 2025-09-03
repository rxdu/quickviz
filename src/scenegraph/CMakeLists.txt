# SceneGraph Module - Modal scene state management with undo/redo
# Supports Direct (zero overhead), Immediate (state tracked), and Recorded (full history) modes

# Find dependencies
find_package(Threads REQUIRED)

# Add scenegraph library
add_library(scenegraph
    # Command pattern implementation
    src/command/command.cpp
    src/command/command_stack.cpp
    
    # Scene state management
    src/state/scene_state.cpp
)

# Link dependencies
target_link_libraries(scenegraph PUBLIC 
    Threads::Threads
    core
    gldraw  # For OpenGlObject interface
)

# Include directories
target_include_directories(scenegraph PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src
)

# Testing
if(BUILD_TESTING)
    add_subdirectory(test)
endif()

# Installation
install(TARGETS scenegraph
    EXPORT quickvizTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)