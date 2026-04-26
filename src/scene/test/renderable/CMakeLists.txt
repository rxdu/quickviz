add_executable(test_arrow test_arrow.cpp)
target_link_libraries(test_arrow PRIVATE scene)

add_executable(test_billboard test_billboard.cpp)
target_link_libraries(test_billboard PRIVATE scene)

add_executable(test_bounding_box test_bounding_box.cpp)
target_link_libraries(test_bounding_box PRIVATE scene)

add_executable(test_canvas test_canvas.cpp)
target_link_libraries(test_canvas PRIVATE scene)

add_executable(test_coordinate_frame test_coordinate_frame.cpp)
target_link_libraries(test_coordinate_frame PRIVATE scene)

add_executable(test_cylinder test_cylinder.cpp)
target_link_libraries(test_cylinder PRIVATE scene)

add_executable(test_frustum test_frustum.cpp)
target_link_libraries(test_frustum PRIVATE scene)

add_executable(test_grid test_grid.cpp)
target_link_libraries(test_grid PRIVATE scene)

add_executable(test_occupancy_grid test_occupancy_grid.cpp)
target_link_libraries(test_occupancy_grid PRIVATE scene)

add_executable(test_line_strip test_line_strip.cpp)
target_link_libraries(test_line_strip PRIVATE scene)

add_executable(test_mesh test_mesh.cpp)
target_link_libraries(test_mesh PRIVATE scene)

add_executable(test_point_cloud test_point_cloud.cpp)
target_link_libraries(test_point_cloud PRIVATE scene)

add_executable(test_path test_path.cpp)
target_link_libraries(test_path PRIVATE scene)

add_executable(test_plane test_plane.cpp)
target_link_libraries(test_plane PRIVATE scene)

add_executable(test_pose test_pose.cpp)
target_link_libraries(test_pose PRIVATE scene)

add_executable(test_sphere test_sphere.cpp)
target_link_libraries(test_sphere PRIVATE scene)


add_executable(test_texture test_texture.cpp)
target_link_libraries(test_texture PRIVATE scene)

add_executable(test_triangle test_triangle.cpp)
target_link_libraries(test_triangle PRIVATE scene)

