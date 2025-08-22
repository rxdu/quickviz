# add unit tests
add_executable(gldraw_unit_tests
    test_pcl_loader.cpp)
target_link_libraries(gldraw_unit_tests PRIVATE gtest_main gmock imview gldraw visualization ${PCL_LIBRARIES})
# get_target_property(PRIVATE_HEADERS imview INCLUDE_DIRECTORIES)
target_include_directories(gldraw_unit_tests PRIVATE ${PRIVATE_HEADERS})

gtest_discover_tests(gldraw_unit_tests)
add_test(NAME gtest_all COMMAND gldraw_unit_tests)
