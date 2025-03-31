# find dependency
find_package(PkgConfig REQUIRED)
pkg_check_modules(Cairo REQUIRED IMPORTED_TARGET cairo)
pkg_check_modules(Fontconfig REQUIRED IMPORTED_TARGET fontconfig)

find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)

# (optional) OpenCV support
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
  message(STATUS "OpenCV found: ${OpenCV_VERSION}")
  set(ENABLE_OPENCV_SUPPORT ON)
  set(OPENCV_COMP_SRC
      src/details/image_utils.cpp
      src/cv_image_widget.cpp
      src/buffered_cv_image_widget.cpp)
  set(OPENCV_COMP_LIBS ${OpenCV_LIBS})
else()
  message(STATUS "OpenCV not found")
endif()

# add library
add_library(widget
    # widgets
    src/details/cairo_context.cpp
    src/details/cairo_draw.cpp
    src/details/scrolling_plot_buffer.cpp
    src/cairo_widget.cpp
    src/rt_line_plot_widget.cpp
    ${OPENCV_COMP_SRC})
target_link_libraries(widget PUBLIC
    core
    imcore imview
    stb
    PkgConfig::Cairo PkgConfig::Fontconfig
    Threads::Threads
    OpenGL::GL
    ${OPENCV_COMP_LIBS})
target_include_directories(widget PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

install(TARGETS widget
    EXPORT quickvizTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
