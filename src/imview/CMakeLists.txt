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
    src/component/image_utils.cpp
    src/widget/cv_image_widget.cpp
    src/widget/buffered_cv_image_widget.cpp)
  set(OPENCV_COMP_LIBS ${OpenCV_LIBS})
else()
  message(STATUS "OpenCV not found")
endif()

# (optional) auto layout and TUI support
if(ENABLE_AUTO_LAYOUT)
  set(AUTO_LAYOUT_SRC
    src/box.cpp
    src/yoga_utils.cpp)
  set(AUTO_LAYOUT_LIBS yogacore)

  set(CURSES_NEED_NCURSES TRUE)
  find_package(Curses QUIET)
  if(CURSES_FOUND)
    set(ENABLE_TUI_SUPPORT ON)
  endif()

  if(ENABLE_TUI_SUPPORT)
    set(TUI_COMP_SRC
      src/terminal/tui_text.cpp
      src/terminal/tui_panel.cpp
      src/terminal/tui_viewer.cpp)
    set(TUI_COMP_LIBS ${CURSES_LIBRARIES})
  endif()
endif()

# add library
add_library(imview
  # ui components
  src/window.cpp
  src/viewer.cpp
  src/popup.cpp
  src/fonts.cpp
  src/scene_object.cpp
  src/panel.cpp
  ${AUTO_LAYOUT_SRC}
  # widgets
  src/widget/cairo_widget.cpp
  src/widget/rt_line_plot_widget.cpp
  ${OPENCV_COMP_SRC}
  # components
  src/component/popup_manager.cpp
  src/component/cairo_context.cpp
  src/component/cairo_draw.cpp
  src/component/opengl/shader.cpp
  src/component/opengl/shader_program.cpp
  src/component/opengl/frame_buffer.cpp
  src/component/opengl/renderer/grid.cpp
  src/component/opengl/renderer/triangle.cpp
  src/component/opengl/renderer/point_cloud.cpp
  src/component/opengl/renderer/canvas.cpp
  src/component/opengl/renderer/coordinate_frame.cpp
  src/component/opengl/camera.cpp
  src/component/opengl/camera_controller.cpp
  src/component/opengl/gl_scene_manager.cpp
  src/component/buffer/buffer_registry.cpp
  src/component/buffer/scrolling_plot_buffer.cpp
  src/component/event/event_dispatcher.cpp
  src/component/event/async_event_dispatcher.cpp
  src/component/logging/log_processor.cpp
  src/component/logging/app_log_handler.cpp
  # terminal
  ${TUI_COMP_SRC})
target_link_libraries(imview PUBLIC imcore stb
  PkgConfig::Cairo PkgConfig::Fontconfig
  Threads::Threads
  OpenGL::GL
  ${AUTO_LAYOUT_LIBS}
  ${OPENCV_COMP_LIBS}
  ${TUI_COMP_LIBS})
if(ENABLE_AUTO_LAYOUT)
  target_compile_definitions(imview PUBLIC ENABLE_AUTO_LAYOUT)
endif()
if(IMVIEW_WITH_GLAD)
  target_link_libraries(imview PUBLIC glad)
  target_compile_definitions(imview PUBLIC IMVIEW_WITH_GLAD)
endif()
target_include_directories(imview PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

install(TARGETS imview
  EXPORT quickvizTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
