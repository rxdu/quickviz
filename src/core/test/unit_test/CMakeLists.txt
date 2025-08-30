# add unit tests
add_executable(core_unit_tests
    utest_ring_buffer.cpp
    utest_double_buffer.cpp
    test_input_event.cpp
    test_event_system.cpp)
target_link_libraries(core_unit_tests PRIVATE gtest_main gmock imview)
# get_target_property(PRIVATE_HEADERS imview INCLUDE_DIRECTORIES)
target_include_directories(core_unit_tests PRIVATE ${PRIVATE_HEADERS})

gtest_discover_tests(core_unit_tests)
add_test(NAME gtest_core COMMAND core_unit_tests)
