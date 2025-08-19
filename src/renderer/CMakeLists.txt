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
    ## Selection tools
    src/selection/selection_result.cpp
    src/selection/selection_tools.cpp
    ## PCL bridge utilities (optional, depends on PCL)
    )

# Check for PCL and conditionally compile PCL bridge
find_package(PCL QUIET COMPONENTS common io)
if(PCL_FOUND)
    message(STATUS "Found PCL: ${PCL_VERSION} - enabling PCL bridge utilities")
    target_sources(renderer PRIVATE
        src/pcl_bridge/pcl_conversions.cpp
        src/pcl_bridge/pcl_visualization.cpp
    )
    target_include_directories(renderer PRIVATE ${PCL_INCLUDE_DIRS})
    target_link_libraries(renderer PRIVATE ${PCL_LIBRARIES})
    target_compile_definitions(renderer PUBLIC QUICKVIZ_WITH_PCL PRIVATE ${PCL_DEFINITIONS})
else()
    message(STATUS "PCL not found - PCL bridge utilities will not be available")
endif()
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
