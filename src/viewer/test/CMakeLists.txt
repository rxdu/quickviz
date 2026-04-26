## feature test
add_subdirectory(feature)

## additional tests/sample
add_executable(test_window_gl_triangle test_window_gl_triangle.cpp)
target_link_libraries(test_window_gl_triangle PRIVATE viewer)

if (ENABLE_AUTO_LAYOUT)
  add_executable(test_box test_box.cpp)
  target_link_libraries(test_box PRIVATE viewer)

  add_executable(test_box_inside_box test_box_inside_box.cpp)
  target_link_libraries(test_box_inside_box PRIVATE viewer)
endif ()

#add_executable(test_panel test_panel.cpp)
#target_link_libraries(test_panel viewer)
