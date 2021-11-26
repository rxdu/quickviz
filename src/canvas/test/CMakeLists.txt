## base
add_executable(test_window test_window.cpp)
target_link_libraries(test_window canvas)

add_executable(test_viewer test_viewer.cpp)
target_link_libraries(test_viewer canvas)

## implot
add_executable(test_plot_buffer test_plot_buffer.cpp)
target_link_libraries(test_plot_buffer PRIVATE canvas)

## cairo
add_executable(test_cairo_context test_cairo_context.cpp)
target_link_libraries(test_cairo_context PRIVATE canvas)

add_executable(test_cairo_widget test_cairo_widget.cpp)
target_link_libraries(test_cairo_widget canvas)

add_executable(test_cairo_draw test_cairo_draw.cpp)
target_link_libraries(test_cairo_draw canvas)

add_executable(test_cairo_normalize test_cairo_normalize.cpp)
target_link_libraries(test_cairo_normalize canvas)

