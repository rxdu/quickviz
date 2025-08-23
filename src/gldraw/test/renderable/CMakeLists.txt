add_executable(test_arrow test_arrow.cpp)
target_link_libraries(test_arrow PRIVATE gldraw)

add_executable(test_bounding_box test_bounding_box.cpp)
target_link_libraries(test_bounding_box PRIVATE gldraw)

add_executable(test_canvas test_canvas.cpp)
target_link_libraries(test_canvas PRIVATE gldraw)

add_executable(test_coordinate_frame test_coordinate_frame.cpp)
target_link_libraries(test_coordinate_frame PRIVATE gldraw)

add_executable(test_cylinder test_cylinder.cpp)
target_link_libraries(test_cylinder PRIVATE gldraw)

add_executable(test_frustum test_frustum.cpp)
target_link_libraries(test_frustum PRIVATE gldraw)

add_executable(test_grid test_grid.cpp)
target_link_libraries(test_grid PRIVATE gldraw)

add_executable(test_line_strip test_line_strip.cpp)
target_link_libraries(test_line_strip PRIVATE gldraw)

add_executable(test_mesh test_mesh.cpp)
target_link_libraries(test_mesh PRIVATE gldraw)

add_executable(test_point_cloud test_point_cloud.cpp)
target_link_libraries(test_point_cloud PRIVATE gldraw)

add_executable(test_sphere test_sphere.cpp)
target_link_libraries(test_sphere PRIVATE gldraw)

add_executable(test_text3d test_text3d.cpp)
target_link_libraries(test_text3d PRIVATE gldraw)

add_executable(test_texture test_texture.cpp)
target_link_libraries(test_texture PRIVATE gldraw)

add_executable(test_triangle test_triangle.cpp)
target_link_libraries(test_triangle PRIVATE gldraw)

