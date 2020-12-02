# add library
add_library(canvas src/imcanvas.cpp)
target_link_libraries(canvas PUBLIC imtoolkit)
target_include_directories(canvas PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()