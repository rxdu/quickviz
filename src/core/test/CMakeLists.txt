## unit test
add_subdirectory(unit_test)


add_executable(test_event test_event.cpp)
target_link_libraries(test_event PRIVATE viewer)

add_executable(test_async_event test_async_event.cpp)
target_link_libraries(test_async_event PRIVATE viewer)

find_package(OpenCV QUIET)
if (OpenCV_FOUND)
  add_executable(test_double_buffer test_double_buffer.cpp)
  target_link_libraries(test_double_buffer PRIVATE viewer ${OpenCV_LIBS})

  add_executable(test_ring_buffer test_ring_buffer.cpp)
  target_link_libraries(test_ring_buffer PRIVATE viewer ${OpenCV_LIBS})
endif ()