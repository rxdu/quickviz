cmake_minimum_required(VERSION 3.16.0)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Found ccache")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

project(quickviz VERSION 0.2.0)

## Project Options
option(BUILD_TESTING "Build tests" ON)
option(STATIC_CHECK "Perform static check" OFF)
option(IMVIEW_WITH_GLAD "Integrate glad into imview" ON)

## Check if code compiles on x86_64 platform
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
	set(BUILD_ON_X86_64 TRUE)
else()
	set(BUILD_ON_X86_64 FALSE)
endif()

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(STATIC_CHECK)
    find_program(CPPCHECK cppcheck)
    if(CPPCHECK)
        message(STATUS "Found cppcheck")
        set(CMAKE_CXX_CPPCHECK cppcheck;--std=c++11;--enable=all)
    endif() 
endif()

## Additional cmake module path
# include(${CMAKE_BINARY_DIR}/conan_paths.cmake)
set(USER_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${USER_CMAKE_PATH}/modules")
list(APPEND CMAKE_PREFIX_PATH "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/cmake")

## Set compiler to use c++ 17 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Chosse build type
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

## Use GNUInstallDirs to install libraries into correct locations on all platforms.
include(GNUInstallDirs)

## Put all binary files into /bin and libraries into /lib
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

# Build tests
if(PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME AND BUILD_TESTING)
    enable_testing()
    include(GoogleTest)
    # add_subdirectory(tests)
    message(STATUS "Tests will be built")
else()
    message(STATUS "Tests will not be built")
endif()

## Add source directory
add_subdirectory(src)

#add_library(quickviz INTERFACE)
#target_link_libraries(quickviz INTERFACE imcore imview cvdraw)

## Installation setup
message(STATUS "Project will be installed to ${CMAKE_INSTALL_PREFIX} with 'make install'")

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIBDIR ${CMAKE_INSTALL_LIBDIR} CACHE PATH "Installation directory for libraries")
set(INSTALL_BINDIR ${CMAKE_INSTALL_BINDIR} CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDEDIR ${CMAKE_INSTALL_INCLUDEDIR} CACHE PATH "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
    set(DEF_INSTALL_CMAKEDIR CMake)
else()
    set(DEF_INSTALL_CMAKEDIR share/cmake/${PROJECT_NAME})
endif()
set(INSTALL_CMAKEDIR ${DEF_INSTALL_CMAKEDIR} CACHE PATH "Installation directory for CMake files")

# print installation path to user
foreach(p LIB BIN INCLUDE CMAKE)
    file(TO_NATIVE_PATH ${CMAKE_INSTALL_PREFIX}/${INSTALL_${p}DIR} _path)
    message(STATUS "Installation path for ${p}: ${_path}")
    unset(_path)
endforeach()

# targets to install
install(TARGETS quickviz
    EXPORT quickvizTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

# export target configuration 
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    quickvizConfigVersion.cmake
    VERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH}"
    COMPATIBILITY SameMajorVersion)

install(EXPORT quickvizTargets
    FILE quickvizTargets.cmake
    NAMESPACE quickviz::
    DESTINATION lib/cmake/quickviz)

configure_file(cmake/quickvizConfig.cmake.in quickvizConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/quickvizConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/quickvizConfigVersion.cmake"
        DESTINATION lib/cmake/quickviz)

# Packaging support
set(CPACK_PACKAGE_NAME "quickviz")
set(CPACK_PACKAGE_VENDOR "Ruixiang Du")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "QuickViz: a lightweight visualization library for C++")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/quickviz")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")

set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Ruixiang Du") 
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libgl1-mesa-dev, libglfw3-dev, libcairo2-dev")
set(CPACK_SOURCE_IGNORE_FILES .git dist .*build.* \\\\.DS_Store)
include(CPack)