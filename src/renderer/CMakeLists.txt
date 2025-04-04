# find dependency
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)

# add library
add_library(renderer
    src/shader.cpp
    src/shader_program.cpp
    src/frame_buffer.cpp
    src/camera.cpp
    src/camera_controller.cpp
    src/gl_scene_manager.cpp
    ## renderable objects
    src/renderable/grid.cpp
    src/renderable/triangle.cpp
    src/renderable/point_cloud.cpp
    src/renderable/canvas.cpp
    src/renderable/coordinate_frame.cpp
    src/renderable/texture.cpp
    )
target_link_libraries(renderer PUBLIC core imcore imview stb
    Threads::Threads
    OpenGL::GL)
if (IMVIEW_WITH_GLAD)
  target_link_libraries(renderer PUBLIC glad)
  target_compile_definitions(renderer PUBLIC IMVIEW_WITH_GLAD)
endif ()
target_include_directories(renderer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if (BUILD_TESTING)
  add_subdirectory(test)
endif ()

install(TARGETS renderer
    EXPORT quickvizTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
