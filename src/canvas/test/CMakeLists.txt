add_executable(test_canvas test_canvas.cpp)
target_link_libraries(test_canvas PRIVATE canvas)

add_executable(test_cairo test_cairo.cpp)
target_link_libraries(test_cairo PRIVATE canvas)
