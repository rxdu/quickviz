add_executable(test_window test_window.cpp)
target_link_libraries(test_window PRIVATE imview)

if (ENABLE_AUTO_LAYOUT)
  add_executable(test_viewer test_viewer.cpp)
  target_link_libraries(test_viewer PRIVATE imview)

  add_executable(test_auto_layout test_auto_layout.cpp)
  target_link_libraries(test_auto_layout PRIVATE imview)
endif ()

add_executable(test_cv_image_widget test_cv_image_widget.cpp)
target_link_libraries(test_cv_image_widget PRIVATE imview)

add_executable(test_buffered_cv_image_widget test_buffered_cv_image_widget.cpp)
target_link_libraries(test_buffered_cv_image_widget PRIVATE imview)

add_executable(test_cairo_widget test_cairo_widget.cpp)
target_link_libraries(test_cairo_widget PRIVATE imview)

add_executable(test_implot_widget test_implot_widget.cpp)
target_link_libraries(test_implot_widget PRIVATE imview)

add_executable(test_gl_widget test_gl_widget.cpp)
target_link_libraries(test_gl_widget PRIVATE imview)

add_executable(test_popup test_popup.cpp)
target_link_libraries(test_popup PRIVATE imview)
