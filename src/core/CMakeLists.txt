# find dependency
find_package(Threads REQUIRED)

# add library
add_library(core
    # widgets
    src/buffer/buffer_registry.cpp
    src/event/async_event_dispatcher.cpp
    src/event/input_mapping.cpp)
target_link_libraries(core PUBLIC Threads::Threads imcore)
target_include_directories(core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

add_executable(binary_to_compressed_c src/fonts/binary_to_compressed_c.cpp)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

install(TARGETS core
    EXPORT quickvizTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
