# add unit tests
add_executable(scene_unit_tests
    test_pcl_loader.cpp)
target_link_libraries(scene_unit_tests PRIVATE gtest_main gmock viewer scene pcl_bridge ${PCL_LIBRARIES})
# get_target_property(PRIVATE_HEADERS viewer INCLUDE_DIRECTORIES)
target_include_directories(scene_unit_tests PRIVATE ${PRIVATE_HEADERS})

gtest_discover_tests(scene_unit_tests)
add_test(NAME gtest_all COMMAND scene_unit_tests)
