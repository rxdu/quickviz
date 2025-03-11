## feature test
add_subdirectory(feature)

## unit test
add_subdirectory(unit_test)

## additional tests/sample
add_executable(test_window_gl_triangle test_window_gl_triangle.cpp)
target_link_libraries(test_window_gl_triangle PRIVATE imview)

if (ENABLE_AUTO_LAYOUT)
  add_executable(test_box test_box.cpp)
  target_link_libraries(test_box PRIVATE imview)

  add_executable(test_box_inside_box test_box_inside_box.cpp)
  target_link_libraries(test_box_inside_box PRIVATE imview)
endif ()

add_executable(test_event test_event.cpp)
target_link_libraries(test_event PRIVATE imview)

add_executable(test_async_event test_async_event.cpp)
target_link_libraries(test_async_event PRIVATE imview)

add_executable(test_framebuffer test_framebuffer.cpp)
target_link_libraries(test_framebuffer PRIVATE imview)

add_executable(test_shader test_shader.cpp)
target_link_libraries(test_shader PRIVATE imview)

add_executable(test_grid test_grid.cpp)
target_link_libraries(test_grid PRIVATE imview)

add_executable(test_camera test_camera.cpp)
target_link_libraries(test_camera PRIVATE imview)

find_package(OpenCV QUIET)
if (OpenCV_FOUND)
  add_executable(test_double_buffer test_double_buffer.cpp)
  target_link_libraries(test_double_buffer PRIVATE imview ${OpenCV_LIBS})

  add_executable(test_ring_buffer test_ring_buffer.cpp)
  target_link_libraries(test_ring_buffer PRIVATE imview ${OpenCV_LIBS})
endif ()

add_executable(test_point_cloud_buffer_strategies test_point_cloud_buffer_strategies.cpp)
target_link_libraries(test_point_cloud_buffer_strategies PRIVATE imview)

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

