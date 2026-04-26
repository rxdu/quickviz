# core modules
add_subdirectory(core)
add_subdirectory(viewer)
add_subdirectory(widget)
add_subdirectory(scene)
add_subdirectory(plot)
add_subdirectory(pcl_bridge)

find_package(OpenCV QUIET)
if (OpenCV_FOUND)
  add_subdirectory(cvdraw)
endif ()
