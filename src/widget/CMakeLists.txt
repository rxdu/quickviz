# NOTE: this module is being dissolved. Cairo bits moved to src/canvas/;
# OpenCV bits will move to src/image/ in a follow-up commit. After that
# this entire directory will be removed.

find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)

# (optional) OpenCV support
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
  message(STATUS "OpenCV found: ${OpenCV_VERSION}")
  set(ENABLE_OPENCV_SUPPORT ON)
  set(OPENCV_COMP_SRC
      src/details/image_utils.cpp
      src/cv_image_widget.cpp
      src/buffered_cv_image_widget.cpp)
  set(OPENCV_COMP_LIBS ${OpenCV_LIBS})
else()
  message(STATUS "OpenCV not found")
endif()

if(NOT OpenCV_FOUND)
  return()  # nothing left to build in widget/ when OpenCV is absent
endif()

# add library
add_library(widget
    ${OPENCV_COMP_SRC})
target_link_libraries(widget PUBLIC
    core
    imcore viewer
    stb
    Threads::Threads
    OpenGL::GL
    ${OPENCV_COMP_LIBS})
target_include_directories(widget PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

install(TARGETS widget
    EXPORT quickvizTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
