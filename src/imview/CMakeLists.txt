# find dependency
find_package(PkgConfig REQUIRED)
pkg_check_modules(Cairo REQUIRED IMPORTED_TARGET cairo)
pkg_check_modules(Fontconfig REQUIRED IMPORTED_TARGET fontconfig)

find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)

# (optional) auto layout and TUI support
if (ENABLE_AUTO_LAYOUT)
  set(AUTO_LAYOUT_SRC
      src/box.cpp
      src/yoga_utils.cpp)
  set(AUTO_LAYOUT_LIBS yogacore)

  set(CURSES_NEED_NCURSES TRUE)
  find_package(Curses QUIET)
  if (CURSES_FOUND)
    set(ENABLE_TUI_SUPPORT ON)
  endif ()

  if (ENABLE_TUI_SUPPORT)
    set(TUI_COMP_SRC
        src/terminal/tui_text.cpp
        src/terminal/tui_panel.cpp
        src/terminal/tui_viewer.cpp)
    set(TUI_COMP_LIBS ${CURSES_LIBRARIES})
  endif ()
endif ()

# add library
add_library(imview
    # ui components
    src/window.cpp
    src/viewer.cpp
    src/popup.cpp
    src/fonts.cpp
    src/scene_object.cpp
    src/panel.cpp
    src/popup_manager.cpp
    src/logging/log_processor.cpp
    src/logging/app_log_handler.cpp
    src/opengl_capability_checker.cpp
    ${AUTO_LAYOUT_SRC}
    # terminal
    ${TUI_COMP_SRC})
target_link_libraries(imview PUBLIC core imcore stb
    PkgConfig::Cairo PkgConfig::Fontconfig
    Threads::Threads
    OpenGL::GL
    ${AUTO_LAYOUT_LIBS}
    ${OPENCV_COMP_LIBS}
    ${TUI_COMP_LIBS})
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
