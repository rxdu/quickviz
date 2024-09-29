# find dependency
find_package(PkgConfig REQUIRED)
pkg_check_modules(Cairo REQUIRED IMPORTED_TARGET cairo)
pkg_check_modules(Fontconfig REQUIRED IMPORTED_TARGET fontconfig)

# add library
add_library(imview
    ## internal implementations
    #    src/cairo_context.cpp
    #    src/data_buffer.cpp
    ## public class
    src/window.cpp
    src/viewer.cpp
    #    src/panel.cpp
    #    src/popup.cpp
    #    src/cairo_widget.cpp
    #    src/cairo_draw.cpp
)
target_link_libraries(imview PUBLIC imcore PkgConfig::Cairo PkgConfig::Fontconfig)
if (IMVIEW_WITH_GLAD)
  target_link_libraries(imview PUBLIC glad)
  target_compile_definitions(imview PUBLIC IMVIEW_WITH_GLAD)
endif ()
target_include_directories(imview PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if (BUILD_TESTS)
  add_subdirectory(test)
endif ()

install(TARGETS imview
    EXPORT quickvizTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})