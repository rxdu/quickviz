add_executable(imgui_sample imgui_sample.cpp)
target_link_libraries(imgui_sample PRIVATE imtoolkit)

add_executable(implot_sample implot_sample.cpp)
target_link_libraries(implot_sample PRIVATE imtoolkit)
