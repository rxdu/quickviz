add_executable(test_implot_widget test_implot_widget.cpp)
target_link_libraries(test_implot_widget PRIVATE widget)

add_executable(test_cairo_widget test_cairo_widget.cpp)
target_link_libraries(test_cairo_widget PRIVATE widget)

if(ENABLE_OPENCV_SUPPORT)
  add_executable(test_cv_image_widget test_cv_image_widget.cpp)
  target_link_libraries(test_cv_image_widget PRIVATE widget)

  add_executable(test_buffered_cv_image_widget test_buffered_cv_image_widget.cpp)
  target_link_libraries(test_buffered_cv_image_widget PRIVATE widget)
endif()
