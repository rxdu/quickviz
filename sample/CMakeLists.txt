cmake_minimum_required(VERSION 2.8.12)
project(PackageTest CXX)

set(CONAN_CMAKE_SETUP ${CMAKE_BINARY_DIR}/conan_paths.cmake)
if (EXISTS ${CONAN_CMAKE_SETUP} AND NOT IS_DIRECTORY ${CONAN_CMAKE_SETUP})
    include(${CONAN_CMAKE_SETUP})
endif()

set(OpenGL_GL_PREFERENCE GLVND)
find_package(imtoolkit REQUIRED)

find_package(PkgConfig REQUIRED) 
pkg_check_modules(Cairo REQUIRED IMPORTED_TARGET cairo)

add_executable(example sample_cairo_canvas.cpp)
target_link_libraries(example PRIVATE rdu::canvas)

# CTest is a testing tool that can be used to test your project.
# enable_testing()
# add_test(NAME example
#          WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin
#          COMMAND example)
