## Dependency libraries
# dependencies for imgui
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# dependencies for imtui
find_package(Curses QUIET)
if(CURSES_FOUND)
    check_library_exists("${CURSES_LIBRARIES}" nodelay "" CURSES_HAS_NODELAY)
    if(NOT CURSES_HAS_NODELAY)
        message(STATUS "Found curses library but not nodelay")
        find_library(CURSES_EXTRA_LIBRARY tinfo)
        check_library_exists("${CURSES_EXTRA_LIBRARY}" nodelay "" CURSES_TINFO_HAS_NODELAY)
        if(CURSES_EXTRA_LIBRARY)
                set(CURSES_LIBRARIES ${CURSES_LIBRARIES} ${CURSES_EXTRA_LIBRARY})
        endif()
    endif()
endif()

## Add libraries
set(IMGUI_CORE_SRC
    imgui/imgui.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_widgets.cpp
    imgui/imgui_demo.cpp
)
set(IMGUI_BACKEND_SRC
    imgui/examples/imgui_impl_glfw.cpp 
    imgui/examples/imgui_impl_opengl3.cpp
)
set(GL_LOADER_SRC
    imgui/examples/libs/gl3w/GL/gl3w.c
)
set(IMPLOT_SRC
    implot/implot.cpp
    implot/implot_items.cpp
)
if(CURSES_FOUND)
set(IMTUI_SRC 
    imtui/src/imtui-impl-text.cpp 
    imtui/src/imtui-impl-ncurses.cpp)
set(IMTUI_DEP_LIBS ${CURSES_LIBRARIES})
endif()
add_library(imcore ${IMGUI_CORE_SRC} ${IMGUI_BACKEND_SRC} ${GL_LOADER_SRC} ${IMPLOT_SRC} ${IMTUI_SRC})
target_link_libraries(imcore PUBLIC glfw OpenGL::GL ${GLFW3_LIBRARY} ${CMAKE_DL_LIBS} ${IMTUI_DEP_LIBS})
target_compile_definitions(imcore PUBLIC "-DIMGUI_IMPL_OPENGL_LOADER_GL3W")
target_include_directories(imcore PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imgui>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imgui/examples>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imgui/examples/libs/gl3w>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/imtui/include>
    $<INSTALL_INTERFACE:include>)

add_subdirectory(sample)
