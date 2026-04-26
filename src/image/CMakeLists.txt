# `image` provides image display and annotation widgets backed by OpenCV.
# Use this module to show camera frames, debug images, or annotated cv::Mat
# data inside a panel. For 2D vector drawing or charts, see canvas/ or plot/.
#
# OpenCV is required. If absent, the module is skipped quietly.

find_package(OpenCV QUIET)
if(NOT OpenCV_FOUND)
  message(STATUS "OpenCV not found — skipping image/ module")
  return()
endif()

find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)

add_library(image
    src/cv_io.cpp
    src/cv_canvas.cpp
    src/cv_colors.cpp
    src/color_maps.cpp
    src/details/image_utils.cpp
    src/cv_image_widget.cpp
    src/buffered_cv_image_widget.cpp)
target_link_libraries(image PUBLIC
    core
    imcore
    viewer
    stb
    Threads::Threads
    OpenGL::GL
    ${OpenCV_LIBS})
target_include_directories(image PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src ${OpenCV_INCLUDE_DIRS})

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

install(TARGETS image
    EXPORT quickvizTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
