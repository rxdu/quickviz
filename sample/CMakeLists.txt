# applications
if (ENABLE_AUTO_LAYOUT)
  add_subdirectory(quickviz_demo_app)

  # Silence PCL-era policy warnings, but keep modern behavior where safe
  if (POLICY CMP0144)
    cmake_policy(SET CMP0144 NEW)
  endif ()
  if (POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
  endif ()
  find_package(PCL QUIET COMPONENTS common io search segmentation)
  if (PCL_FOUND)
    message(STATUS "Found PCL: ${PCL_VERSION}")

    add_subdirectory(pointcloud_viewer)
  endif ()
endif ()