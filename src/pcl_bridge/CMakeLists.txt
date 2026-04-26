# Create visualization library
add_library(pcl_bridge
    # PCL bridge utilities (optional, depends on PCL)
)

# Check for PCL and conditionally compile PCL bridge
find_package(PCL QUIET COMPONENTS common io)
if(PCL_FOUND)
    message(STATUS "Found PCL: ${PCL_VERSION} - enabling PCL bridge utilities")
    target_sources(pcl_bridge PRIVATE
        src/pcl_conversions.cpp
        src/pcl_visualization.cpp
        src/pcl_loader.cpp
    )
    target_include_directories(pcl_bridge PRIVATE ${PCL_INCLUDE_DIRS})
    target_link_libraries(pcl_bridge PRIVATE ${PCL_LIBRARIES})
    target_compile_definitions(pcl_bridge PUBLIC QUICKVIZ_WITH_PCL PRIVATE ${PCL_DEFINITIONS})
else()
    message(STATUS "PCL not found - PCL bridge utilities will not be available")
endif()

# Link with gldraw for rendering capabilities
target_link_libraries(pcl_bridge PUBLIC gldraw)

# Include directories
target_include_directories(pcl_bridge PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src
)

# Tests for pcl_bridge module
if (BUILD_TESTING)
  add_subdirectory(test)
endif ()

# Installation
install(TARGETS pcl_bridge
    EXPORT quickvizTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)