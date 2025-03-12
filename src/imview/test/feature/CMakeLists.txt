add_executable(test_window test_window.cpp)
target_link_libraries(test_window PRIVATE imview)

if(ENABLE_AUTO_LAYOUT)
  add_executable(test_viewer test_viewer.cpp)
  target_link_libraries(test_viewer PRIVATE imview)

  add_executable(test_auto_layout test_auto_layout.cpp)
  target_link_libraries(test_auto_layout PRIVATE imview)

  if(ENABLE_TUI_SUPPORT)
    add_executable(test_tui_composer test_tui_composer.cpp)
    target_link_libraries(test_tui_composer PRIVATE imview)
  endif()

endif()

if(ENABLE_OPENCV_SUPPORT)
  add_executable(test_cv_image_widget test_cv_image_widget.cpp)
  target_link_libraries(test_cv_image_widget PRIVATE imview)

  add_executable(test_buffered_cv_image_widget test_buffered_cv_image_widget.cpp)
  target_link_libraries(test_buffered_cv_image_widget PRIVATE imview)
endif()

add_executable(test_implot_widget test_implot_widget.cpp)
target_link_libraries(test_implot_widget PRIVATE imview)

add_executable(test_cairo_widget test_cairo_widget.cpp)
target_link_libraries(test_cairo_widget PRIVATE imview)

add_executable(test_gl_scene_manager test_gl_scene_manager.cpp)
target_link_libraries(test_gl_scene_manager PRIVATE imview)

add_executable(test_point_cloud test_point_cloud.cpp)
target_link_libraries(test_point_cloud PRIVATE imview)

add_executable(test_point_cloud_realtime test_point_cloud_realtime.cpp)
target_link_libraries(test_point_cloud_realtime PRIVATE imview)

add_executable(test_popup test_popup.cpp)
target_link_libraries(test_popup PRIVATE imview)

add_executable(test_joystick_input test_joystick_input.cpp)
target_link_libraries(test_joystick_input PRIVATE imview)
