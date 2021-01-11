find_package(Threads REQUIRED)

add_executable(test_data_plotter test_data_plotter.cpp)
target_link_libraries(test_data_plotter PRIVATE plotter Threads::Threads)
