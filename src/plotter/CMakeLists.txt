# add library
add_library(plotter 
    src/data_plotter.cpp)
target_link_libraries(plotter PUBLIC imcore canvas)
target_include_directories(plotter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()