# core modules
add_subdirectory(core)
add_subdirectory(imview)
add_subdirectory(widget)
add_subdirectory(gldraw)
add_subdirectory(visualization)

find_package(OpenCV QUIET)
if (OpenCV_FOUND)
  add_subdirectory(cvdraw)
endif ()
