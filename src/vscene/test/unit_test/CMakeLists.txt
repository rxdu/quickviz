# add unit tests
add_executable(vscene_unit_tests
    utest_virtual_object.cpp
    utest_virtual_scene.cpp
    utest_object_types.cpp
    utest_virtual_sphere_integration.cpp
    utest_event_system.cpp
    utest_virtual_scene_panel.cpp
    utest_render_backend.cpp)
target_link_libraries(vscene_unit_tests PRIVATE gtest_main gmock vscene)
# get_target_property(PRIVATE_HEADERS imview INCLUDE_DIRECTORIES)
target_include_directories(vscene_unit_tests PRIVATE ${PRIVATE_HEADERS})

gtest_discover_tests(vscene_unit_tests)
add_test(NAME gtest_vscene COMMAND vscene_unit_tests)
