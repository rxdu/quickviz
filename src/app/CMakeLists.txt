if (BUILD_QUICKVIZ_APP)
  message(STATUS "Build quickviz application")
  add_executable(quickviz main.cpp
      quickviz_application.cpp
      # panels
      panels/menu_bar.cpp
      panels/main_docking_panel.cpp
      panels/scene_panel.cpp
      panels/config_panel.cpp
      panels/console_panel.cpp)
  target_link_libraries(quickviz PRIVATE imview renderer)
  target_include_directories(quickviz PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

  install(TARGETS quickviz
      EXPORT quickvizTargets
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
      RUNTIME DESTINATION bin
      INCLUDES DESTINATION include)
endif ()