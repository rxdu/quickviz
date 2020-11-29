cmake_minimum_required(VERSION 3.16.0)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Found ccache")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

project(imtoolkit VERSION 0.1.0)

set(BUILD_TESTING on)

## Project Options
option(BUILD_TESTING ON)
option(STATIC_CHECK OFF)
option(ENABLE_LOGGING ON)

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

## Set compiler to use c++ 14 features
set(CMAKE_CXX_STANDARD 14)
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
    set(BUILD_TESTS ON)
    # add_subdirectory(tests)
    message(STATUS "Tests will be built")
else()
    set(BUILD_TESTS OFF)
    message(STATUS "Tests will not be built")
endif()

## Add source directory
add_subdirectory(src)

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
install(TARGETS imtoolkit
    EXPORT imtoolkitTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY src/imgui src/implot
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    PATTERN "*.h")

# export target configuration 
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    imtoolkitConfigVersion.cmake
    VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}"
    COMPATIBILITY SameMajorVersion)

install(EXPORT imtoolkitTargets
    FILE imtoolkitTargets.cmake
    NAMESPACE rdu::
    DESTINATION lib/cmake/imtoolkit)

configure_file(cmake/imtoolkitConfig.cmake.in imtoolkitConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/imtoolkitConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/imtoolkitConfigVersion.cmake"
        DESTINATION lib/cmake/imtoolkit)

# Packaging support
set(CPACK_PACKAGE_VENDOR "Ruixiang Du")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Immediate-Mode Toolkit")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
# set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/imtoolkit")
# set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENCE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")

set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Ruixiang Du") 
# set(CPACK_DEBIAN_PACKAGE_DEPENDS )
set(CPACK_SOURCE_IGNORE_FILES .git dist .*build.* \\\\.DS_Store)
include(CPack)