add_subdirectory(feature)

add_executable(test_framebuffer test_framebuffer.cpp)
target_link_libraries(test_framebuffer PRIVATE renderer)

add_executable(test_shader test_shader.cpp)
target_link_libraries(test_shader PRIVATE renderer)

add_executable(test_grid test_grid.cpp)
target_link_libraries(test_grid PRIVATE renderer)

add_executable(test_camera test_camera.cpp)
target_link_libraries(test_camera PRIVATE renderer)

add_executable(test_point_cloud_realtime test_point_cloud_realtime.cpp)
target_link_libraries(test_point_cloud_realtime PRIVATE renderer)

add_executable(test_point_cloud_buffer_strategies test_point_cloud_buffer_strategies.cpp)
target_link_libraries(test_point_cloud_buffer_strategies PRIVATE renderer)

add_executable(test_coordinate_system test_coordinate_system.cpp)
target_link_libraries(test_coordinate_system PRIVATE renderer)

add_executable(test_primitive_drawing test_primitive_drawing.cpp)
target_link_libraries(test_primitive_drawing PRIVATE renderer)

add_executable(test_texture test_texture.cpp)
target_link_libraries(test_texture PRIVATE renderer)

add_executable(test_canvas_st test_canvas_st.cpp)
target_link_libraries(test_canvas_st PRIVATE renderer)

add_executable(test_nav_map_rendering test_nav_map_rendering.cpp)
target_link_libraries(test_nav_map_rendering PRIVATE renderer)

add_executable(test_camera_enhanced test_camera_enhanced.cpp)
target_link_libraries(test_camera_enhanced PRIVATE renderer)

# Silence PCL-era policy warnings, but keep modern behavior where safe
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()
find_package(PCL QUIET COMPONENTS common io)
if(PCL_FOUND)
  message(STATUS "Found PCL: ${PCL_VERSION}")
  add_executable(test_pcd test_pcd.cpp)
  target_include_directories(test_pcd PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_pcd PRIVATE renderer ${PCL_LIBRARIES})
  target_compile_definitions(test_pcd PRIVATE ${PCL_DEFINITIONS})
endif()