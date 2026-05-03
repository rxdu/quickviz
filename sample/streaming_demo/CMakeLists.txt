add_executable(quickviz_streaming_demo main.cpp)
target_link_libraries(quickviz_streaming_demo PRIVATE
    core
    scene
    viewer)
