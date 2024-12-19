# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## tests
add_executable(test_image_rw test_image_rw.cpp)
target_link_libraries(test_image_rw PUBLIC cvdraw)

add_executable(test_cv_canvas test_cv_canvas.cpp)
target_link_libraries(test_cv_canvas PUBLIC cvdraw)

add_executable(test_cv_drawmode test_cv_drawmode.cpp)
target_link_libraries(test_cv_drawmode PUBLIC cvdraw)

add_executable(test_cv_drawfunc test_cv_drawfunc.cpp)
target_link_libraries(test_cv_drawfunc PUBLIC cvdraw)