add_executable(quickviz_editor
    main.cpp
    editor_state.cpp
    editor_viewport.cpp
    commands/command_stack.cpp
    commands/delete_points_command.cpp
    panels/history_panel.cpp
    panels/editor_tool_panel.cpp)

target_compile_definitions(quickviz_editor PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(quickviz_editor PRIVATE
    scene
    pcl_bridge
    ${PCL_LIBRARIES})
target_include_directories(quickviz_editor PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR})
