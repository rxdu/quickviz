add_library(glad STATIC src/glad.c)
target_link_libraries(glad PUBLIC ${CMAKE_DL_LIBS})
target_include_directories(glad PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

install(TARGETS glad
    EXPORT quickvizTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})