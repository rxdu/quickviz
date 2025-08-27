# Unit tests
add_subdirectory(unit_test)

add_executable(test_virtual_scene test_virtual_scene.cpp)
target_link_libraries(test_virtual_scene vscene imview gldraw)
# This test is expected to fail compilation initially
# It demonstrates the intended API and workflow
set_target_properties(test_virtual_scene PROPERTIES
    EXCLUDE_FROM_ALL TRUE  # Don't build by default
)

add_executable(test_virtual_sphere test_virtual_sphere.cpp)
target_link_libraries(test_virtual_sphere vscene imview gldraw)