add_executable(test_window test_window.cpp)
target_link_libraries(test_window PRIVATE imview)

if(ENABLE_AUTO_LAYOUT)
  add_executable(test_viewer test_viewer.cpp)
  target_link_libraries(test_viewer PRIVATE imview)

  add_executable(test_auto_layout test_auto_layout.cpp)
  target_link_libraries(test_auto_layout PRIVATE imview)

  if(ENABLE_TUI_SUPPORT)
    add_executable(test_tui_composer test_tui_composer.cpp)
    target_link_libraries(test_tui_composer PRIVATE imview)
  endif()
endif()

add_executable(test_popup test_popup.cpp)
target_link_libraries(test_popup PRIVATE imview)

add_executable(test_joystick_input test_joystick_input.cpp)
target_link_libraries(test_joystick_input PRIVATE imview)
