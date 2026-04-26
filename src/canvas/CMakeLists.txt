# find dependency
find_package(PkgConfig REQUIRED)
pkg_check_modules(Cairo REQUIRED IMPORTED_TARGET cairo)
pkg_check_modules(Fontconfig REQUIRED IMPORTED_TARGET fontconfig)

find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)

# add library
#
# `canvas` provides 2D vector drawing and immediate widgets backed by
# Cairo. Use this module for custom 2D figures, plots-as-pictures, and
# annotations rendered into a panel as a texture.
add_library(canvas
    src/details/cairo_context.cpp
    src/details/cairo_draw.cpp
    src/cairo_widget.cpp)
target_link_libraries(canvas PUBLIC
    core
    imcore
    viewer
    PkgConfig::Cairo
    PkgConfig::Fontconfig
    Threads::Threads
    OpenGL::GL)
target_include_directories(canvas PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

install(TARGETS canvas
    EXPORT quickvizTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
