# ==============================================================================
# Scene Interaction Selection Tests
# ==============================================================================
# Modular selection testing for different renderable object types

# Shared utilities for all selection tests
add_library(selection_test_utils selection_test_utils.cpp selection_test_utils.hpp)
target_link_libraries(selection_test_utils PUBLIC gldraw)

# Original comprehensive test (kept for backward compatibility)
#add_executable(test_object_selection test_object_selection.cpp)
#target_link_libraries(test_object_selection PRIVATE gldraw)

# Individual object type selection tests
add_executable(test_sphere_selection test_sphere_selection.cpp)
target_link_libraries(test_sphere_selection PRIVATE selection_test_utils)

add_executable(test_point_cloud_selection test_point_cloud_selection.cpp)  
target_link_libraries(test_point_cloud_selection PRIVATE selection_test_utils)

add_executable(test_line_strip_selection test_line_strip_selection.cpp)
target_link_libraries(test_line_strip_selection PRIVATE selection_test_utils)

add_executable(test_mesh_selection test_mesh_selection.cpp)
target_link_libraries(test_mesh_selection PRIVATE selection_test_utils)

add_executable(test_cylinder_selection test_cylinder_selection.cpp)
target_link_libraries(test_cylinder_selection PRIVATE selection_test_utils)

add_executable(test_bounding_box_selection test_bounding_box_selection.cpp)
target_link_libraries(test_bounding_box_selection PRIVATE selection_test_utils)
