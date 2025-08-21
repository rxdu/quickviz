add_executable(test_pcl_selector
    test_pcl_selector.cpp
    point_cloud_info_panel.cpp
    point_cloud_tool_panel.cpp
    interactive_scene_manager.cpp)
target_compile_definitions(test_pcl_selector PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(test_pcl_selector PRIVATE gldraw ${PCL_LIBRARIES})
target_include_directories(test_pcl_selector PRIVATE ${PCL_INCLUDE_DIRS} ${CURRENT_CMAKE_SOURCE_DIR})
