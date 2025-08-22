add_executable(point_cloud_viewer
    main.cpp
    point_cloud_info_panel.cpp
    point_cloud_tool_panel.cpp
    interactive_scene_manager.cpp)
target_compile_definitions(point_cloud_viewer PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(point_cloud_viewer PRIVATE gldraw visualization ${PCL_LIBRARIES})
target_include_directories(point_cloud_viewer PRIVATE ${PCL_INCLUDE_DIRS} ${CURRENT_CMAKE_SOURCE_DIR})
