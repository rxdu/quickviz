add_executable(quickviz_quickstart main.cpp)
target_link_libraries(quickviz_quickstart PRIVATE
    core
    scene
    viewer)
