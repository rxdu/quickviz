add_subdirectory(feature)

add_executable(test_framebuffer test_framebuffer.cpp)
target_link_libraries(test_framebuffer PRIVATE gldraw)

add_executable(test_shader test_shader.cpp)
target_link_libraries(test_shader PRIVATE gldraw)

add_executable(test_grid test_grid.cpp)
target_link_libraries(test_grid PRIVATE gldraw)

add_executable(test_camera_raw test_camera_raw.cpp)
target_link_libraries(test_camera_raw PRIVATE gldraw)

add_executable(test_point_cloud_realtime test_point_cloud_realtime.cpp)
target_link_libraries(test_point_cloud_realtime PRIVATE gldraw)

add_executable(test_point_cloud_buffer_strategies test_point_cloud_buffer_strategies.cpp)
target_link_libraries(test_point_cloud_buffer_strategies PRIVATE gldraw)

# Test new visualization contracts (now requires visualization module)
add_executable(test_visualization_contracts test_visualization_contracts.cpp)
target_link_libraries(test_visualization_contracts PRIVATE gldraw visualization)

# Test external selection demo (replaces old selection integration)
# test_pcd_with_selection.cpp now demonstrates external selection visualization

add_executable(test_coordinate_system test_coordinate_system.cpp)
target_link_libraries(test_coordinate_system PRIVATE gldraw)

add_executable(test_primitive_drawing test_primitive_drawing.cpp)
target_link_libraries(test_primitive_drawing PRIVATE gldraw)

add_executable(test_texture test_texture.cpp)
target_link_libraries(test_texture PRIVATE gldraw)

add_executable(test_canvas_st test_canvas_st.cpp)
target_link_libraries(test_canvas_st PRIVATE gldraw)

add_executable(test_nav_map_rendering test_nav_map_rendering.cpp)
target_link_libraries(test_nav_map_rendering PRIVATE gldraw)

add_executable(test_layer_system test_layer_system.cpp)
target_link_libraries(test_layer_system PRIVATE gldraw)

# Silence PCL-era policy warnings, but keep modern behavior where safe
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()
find_package(PCL QUIET COMPONENTS common io search segmentation)
if(PCL_FOUND)
  message(STATUS "Found PCL: ${PCL_VERSION}")

  add_executable(test_pcd test_pcd.cpp)
  target_include_directories(test_pcd PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_pcd PRIVATE gldraw ${PCL_LIBRARIES})
  target_compile_definitions(test_pcd PRIVATE ${PCL_DEFINITIONS})
  
  add_executable(test_pcl_bridge test_pcl_bridge.cpp)
  target_include_directories(test_pcl_bridge PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_pcl_bridge PRIVATE gldraw visualization ${PCL_LIBRARIES})
  target_compile_definitions(test_pcl_bridge PRIVATE ${PCL_DEFINITIONS})
  
  add_executable(test_pcd_with_selection test_pcd_with_selection.cpp)
  target_include_directories(test_pcd_with_selection PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_pcd_with_selection PRIVATE gldraw visualization ${PCL_LIBRARIES})
  target_compile_definitions(test_pcd_with_selection PRIVATE ${PCL_DEFINITIONS})
  
#  add_executable(test_pcl_loader unit_test/test_pcl_loader.cpp)
#  target_include_directories(test_pcl_loader PRIVATE ${PCL_INCLUDE_DIRS})
#  target_link_libraries(test_pcl_loader PRIVATE gldraw ${PCL_LIBRARIES} gtest_main)
#  target_compile_definitions(test_pcl_loader PRIVATE ${PCL_DEFINITIONS})
  
  add_executable(test_pcl_loader_render test_pcl_loader_render.cpp)
  target_include_directories(test_pcl_loader_render PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_pcl_loader_render PRIVATE gldraw visualization ${PCL_LIBRARIES})
  target_compile_definitions(test_pcl_loader_render PRIVATE ${PCL_DEFINITIONS})

  add_subdirectory(unit_test)
endif()
