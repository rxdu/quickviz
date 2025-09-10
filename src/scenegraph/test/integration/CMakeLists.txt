# SceneGraph integration tests

# Scene Manager Integration Test
add_executable(test_scene_manager_integration
    test_scene_manager_integration.cpp
)

target_link_libraries(test_scene_manager_integration
    gtest_main
    scenegraph
    gldraw
    imview
    glfw
    glad
)

# Add test to CTest
add_test(NAME SceneManagerIntegration COMMAND test_scene_manager_integration)