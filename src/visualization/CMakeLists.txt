# Create visualization library
add_library(visualization
    # Renderable classes (data to OpenGL object conversion)
    src/renderables/selection_renderable.cpp
    src/renderables/surface_renderable.cpp
    
    # Selection utilities
    src/selection/point_cloud_selector.cpp
    
    # Helper utilities
    src/helpers/visualization_helpers.cpp
    src/helpers/selection_visualizer.cpp
    
    # Testing utilities  
    src/testing/mock_data_generator.cpp
    
    # PCL bridge utilities (optional, depends on PCL)
)

# Check for PCL and conditionally compile PCL bridge
find_package(PCL QUIET COMPONENTS common io)
if(PCL_FOUND)
    message(STATUS "Found PCL: ${PCL_VERSION} - enabling PCL bridge utilities in visualization module")
    target_sources(visualization PRIVATE
        src/pcl_bridge/pcl_conversions.cpp
        src/pcl_bridge/pcl_visualization.cpp
        src/pcl_bridge/pcl_loader.cpp
    )
    target_include_directories(visualization PRIVATE ${PCL_INCLUDE_DIRS})
    target_link_libraries(visualization PRIVATE ${PCL_LIBRARIES})
    target_compile_definitions(visualization PUBLIC QUICKVIZ_WITH_PCL PRIVATE ${PCL_DEFINITIONS})
else()
    message(STATUS "PCL not found - PCL bridge utilities will not be available in visualization module")
endif()

# Link with gldraw for rendering capabilities
target_link_libraries(visualization PUBLIC gldraw)

# Include directories
target_include_directories(visualization PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src
)

# Tests for visualization module
if (BUILD_TESTING)
  add_subdirectory(test)
endif ()

# Installation
install(TARGETS visualization
    EXPORT quickvizTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)