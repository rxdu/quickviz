add_executable(test_gl_scene_manager test_gl_scene_manager.cpp)
target_link_libraries(test_gl_scene_manager PRIVATE renderer)

add_executable(test_point_cloud test_point_cloud.cpp)
target_link_libraries(test_point_cloud PRIVATE renderer)

add_executable(test_coordinate_frame test_coordinate_frame.cpp)
target_link_libraries(test_coordinate_frame PRIVATE renderer)

add_executable(test_robot_frames test_robot_frames.cpp)
target_link_libraries(test_robot_frames PRIVATE renderer)

add_executable(test_canvas test_canvas.cpp)
target_link_libraries(test_canvas PRIVATE renderer)
