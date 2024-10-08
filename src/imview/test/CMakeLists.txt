## feature test
add_subdirectory(feature)

## additional tests/sample
add_executable(test_window_gl_triangle test_window_gl_triangle.cpp)
target_link_libraries(test_window_gl_triangle PRIVATE imview)

add_executable(test_box test_box.cpp)
target_link_libraries(test_box PRIVATE imview)

add_executable(test_event test_event.cpp)
target_link_libraries(test_event PRIVATE imview)

add_executable(test_async_event test_async_event.cpp)
target_link_libraries(test_async_event PRIVATE imview)

find_package(OpenCV QUIET)
if (OpenCV_FOUND)
  add_executable(test_double_buffer test_double_buffer.cpp)
  target_link_libraries(test_double_buffer PRIVATE imview ${OpenCV_LIBS})
endif ()

#add_executable(test_panel test_panel.cpp)
#target_link_libraries(test_panel imview)
#
### implot
#add_executable(test_plot_buffer test_plot_buffer.cpp)
#target_link_libraries(test_plot_buffer PRIVATE imview)
#
### opengl
#add_executable(test_opengl test_opengl.cpp)
#target_link_libraries(test_opengl imview)
#
### cairo
#add_executable(test_cairo_context test_cairo_context.cpp)
#target_link_libraries(test_cairo_context PRIVATE imview)
#
#add_executable(test_cairo_widget test_cairo_widget.cpp)
#target_link_libraries(test_cairo_widget imview)
#
#add_executable(test_cairo_draw test_cairo_draw.cpp)
#target_link_libraries(test_cairo_draw imview)
#
#add_executable(test_cairo_normalize test_cairo_normalize.cpp)
#target_link_libraries(test_cairo_normalize imview)

