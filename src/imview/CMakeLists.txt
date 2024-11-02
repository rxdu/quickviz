# find dependency
find_package(PkgConfig REQUIRED)
pkg_check_modules(Cairo REQUIRED IMPORTED_TARGET cairo)
pkg_check_modules(Fontconfig REQUIRED IMPORTED_TARGET fontconfig)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)

if (ENABLE_AUTO_LAYOUT)
  set(AUTO_LAYOUT_SRC
      src/box.cpp
      src/yoga_utils.cpp)
  set(AUTO_LAYOUT_LIBS yogacore)
endif ()

# add library
add_library(imview
    # ui components
    src/window.cpp
    src/viewer.cpp
    src/fonts.cpp
    src/scene_object.cpp
    src/panel.cpp
    ${AUTO_LAYOUT_SRC}
    src/layer.cpp
    #    src/popup.cpp
    src/buffer/scrolling_plot_buffer.cpp
    # utils
    src/utils/image_utils.cpp
    # widgets
    src/widget/cv_image_widget.cpp
    src/widget/buffered_cv_image_widget.cpp
    src/widget/cairo_widget.cpp
    src/widget/details/cairo_context.cpp
    src/widget/details/cairo_draw.cpp
    src/widget/rt_line_plot_widget.cpp
    src/widget/gl_scene_widget.cpp
    src/widget/details/gl_frame_buffer.cpp
    # data buffer
    src/buffer/buffer_registry.cpp
    # event handling
    src/event/event_dispatcher.cpp
    src/event/async_event_dispatcher.cpp
)
target_link_libraries(imview PUBLIC imcore
    PkgConfig::Cairo PkgConfig::Fontconfig
    OpenGL::OpenGL ${CMAKE_DL_LIBS}
    ${AUTO_LAYOUT_LIBS} ${OpenCV_LIBS})
if (ENABLE_AUTO_LAYOUT)
  target_compile_definitions(imview PUBLIC ENABLE_AUTO_LAYOUT)
endif ()
if (IMVIEW_WITH_GLAD)
  target_link_libraries(imview PUBLIC glad)
  target_compile_definitions(imview PUBLIC IMVIEW_WITH_GLAD)
endif ()
target_include_directories(imview PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if (BUILD_TESTING)
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
