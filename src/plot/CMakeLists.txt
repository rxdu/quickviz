# find dependency
find_package(Threads REQUIRED)

# add library
#
# `plot` provides QuickViz wrappers around ImPlot (2D) and ImPlot3D (3D).
# The umbrella headers `plot/plot2d.hpp` and `plot/plot3d.hpp` simply
# re-export the underlying upstream APIs; concrete widgets like
# `rt_line_plot_widget` build on top of them.
add_library(plot
    src/details/scrolling_plot_buffer.cpp
    src/rt_line_plot_widget.cpp)
target_link_libraries(plot PUBLIC
    core
    imcore
    viewer
    Threads::Threads)
target_include_directories(plot PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

install(TARGETS plot
    EXPORT quickvizTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
