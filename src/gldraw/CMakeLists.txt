# find dependency
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)

# add library
add_library(gldraw
    src/shader.cpp
    src/shader_program.cpp
    src/frame_buffer.cpp
    src/camera.cpp
    src/camera_controller.cpp
    src/gl_scene_manager.cpp
    src/gl_view.cpp
    ## renderable objects
    src/renderable/grid.cpp
    src/renderable/triangle.cpp
    src/renderable/point_cloud.cpp
    src/renderable/canvas.cpp
    src/renderable/details/batched_render_strategy.cpp
    src/renderable/details/individual_render_strategy.cpp
    src/renderable/details/shape_renderer.cpp
    src/renderable/details/shape_generators.cpp
    src/renderable/details/opengl_resource_pool.cpp
    src/renderable/details/shape_renderer_utils.cpp
    src/renderable/details/canvas_data_manager.cpp
    src/renderable/details/data_aware_render_strategy.cpp
    src/renderable/coordinate_frame.cpp
    src/renderable/texture.cpp
    src/renderable/layer_manager.cpp
    src/renderable/mesh.cpp
    src/renderable/line_strip.cpp
    src/renderable/arrow.cpp
    src/renderable/bounding_box.cpp
    src/renderable/sphere.cpp
    src/renderable/cylinder.cpp
    src/renderable/text3d.cpp
    src/renderable/frustum.cpp
    src/renderable/plane.cpp
    src/renderable/pose.cpp
    src/renderable/path.cpp
    src/renderable/vector_field.cpp
    src/renderable/occupancy_grid.cpp
    src/renderable/measurement.cpp
    src/renderable/uncertainty_ellipse.cpp
    src/renderable/sensor_coverage.cpp
    )
target_link_libraries(gldraw PUBLIC core imcore imview stb
    Threads::Threads
    OpenGL::GL)
if (IMVIEW_WITH_GLAD)
  target_link_libraries(gldraw PUBLIC glad)
  target_compile_definitions(gldraw PUBLIC IMVIEW_WITH_GLAD)
endif ()
target_include_directories(gldraw PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if (BUILD_TESTING)
  add_subdirectory(test)
endif ()

install(TARGETS gldraw
    EXPORT quickvizTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
