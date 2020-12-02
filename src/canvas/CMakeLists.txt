# find dependency
find_package(PkgConfig REQUIRED) 
pkg_check_modules(Cairo REQUIRED IMPORTED_TARGET cairo)

# add library
add_library(canvas src/imcanvas.cpp)
target_link_libraries(canvas PUBLIC imtoolkit PkgConfig::Cairo)
target_include_directories(canvas PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()