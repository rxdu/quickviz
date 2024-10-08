add_executable(quickviz main.cpp
    # components
    quickviz_application.cpp
    # panels
    panels/main_docking_panel.cpp
    panels/config_panel.cpp
    panels/scene_panel.cpp
    panels/config_panel.cpp
    panels/console_panel.cpp)
target_link_libraries(quickviz PRIVATE imview)
target_include_directories(quickviz PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
