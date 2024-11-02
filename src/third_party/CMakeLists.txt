# external libraries
if (ENABLE_AUTO_LAYOUT)
  add_subdirectory(yoga)
endif ()
add_subdirectory(glad)
add_subdirectory(imcore)

if (BUILD_TESTING)
  if (TARGET gtest)
    message(STATUS "GoogleTest already found, project will use the existing target")
  else ()
    add_subdirectory(googletest)
  endif ()
endif ()