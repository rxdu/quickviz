# external libraries
add_subdirectory(yoga)
add_subdirectory(glad)
add_subdirectory(imcore)

if (BUILD_TESTING)
  add_subdirectory(googletest)
endif ()