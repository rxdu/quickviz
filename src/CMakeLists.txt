add_subdirectory(imcore)
add_subdirectory(imview)

find_package(OpenCV QUIET)
if(OpenCV_FOUND)
    add_subdirectory(cvdraw)
endif()
