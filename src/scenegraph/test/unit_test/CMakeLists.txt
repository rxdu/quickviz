# SceneGraph unit tests
add_executable(scenegraph_unit_tests
    test_command.cpp
    test_scene_state.cpp
)

target_link_libraries(scenegraph_unit_tests PRIVATE 
    scenegraph 
    gtest_main 
    gmock
)

target_include_directories(scenegraph_unit_tests PRIVATE ${PRIVATE_HEADERS})

gtest_discover_tests(scenegraph_unit_tests)
add_test(NAME gtest_scenegraph COMMAND scenegraph_unit_tests)