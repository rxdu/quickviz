# Unit tests (automated, no GL dependencies)
add_subdirectory(unit_test)

add_executable(test_virtual_sphere_pick test_virtual_sphere_pick.cpp)
target_link_libraries(test_virtual_sphere_pick vscene imview gldraw)

add_executable(test_virtual_sphere test_virtual_sphere.cpp)
target_link_libraries(test_virtual_sphere vscene imview gldraw)
