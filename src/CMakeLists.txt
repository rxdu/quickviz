# core modules
add_subdirectory(imview)

find_package(OpenCV QUIET)
if (OpenCV_FOUND)
  add_subdirectory(cvdraw)
endif ()

add_subdirectory(third_party)

# applications
if (ENABLE_AUTO_LAYOUT)
  add_subdirectory(app)
endif ()
