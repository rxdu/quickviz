# core modules
add_subdirectory(core)
add_subdirectory(imview)
add_subdirectory(widget)
add_subdirectory(renderer)

add_subdirectory(third_party)

find_package(OpenCV QUIET)
if (OpenCV_FOUND)
  add_subdirectory(cvdraw)
endif ()

# applications
#if (ENABLE_AUTO_LAYOUT)
#  add_subdirectory(app)
#endif ()
