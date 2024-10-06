add_executable(test_window test_window.cpp)
target_link_libraries(test_window PRIVATE imview)

add_executable(test_viewer test_viewer.cpp)
target_link_libraries(test_viewer PRIVATE imview)

add_executable(test_box test_box.cpp)
target_link_libraries(test_box PRIVATE imview)