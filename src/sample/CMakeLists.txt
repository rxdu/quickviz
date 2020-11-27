add_executable(sample main.cpp)
target_link_libraries(sample PRIVATE imtoolkit)
target_compile_definitions(sample PRIVATE "-DIMGUI_IMPL_OPENGL_LOADER_GL3W")
