/*
 * @file test_joystick_input.cpp
 * @date 2/13/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>

#include "imview/viewer.hpp"

#include "scene_objects/imgui_fixed_panel.hpp"

using namespace quickviz;

void JoystickDeviceChangeCallback(const std::vector<JoystickDevice>& devices) {
  std::cout << "Total number of joysticks: " << devices.size() << std::endl;
}

void JoystickInputUpdateCallback(const JoystickInput& input) {
  static uint64_t count = 0;
  std::cout << "Joystick input updated: " << count++ << std::endl;
  // print axis
  std::cout << "Axis: ";
  for (int i = 0; i < input.axes.size(); ++i) {
    std::cout << "Axis " << i << ": " << input.axes[i] << " ";
  }
  std::cout << std::endl;
  // print buttons
  std::cout << "Buttons: ";
  for (int i = 0; i < input.buttons.size(); ++i) {
    std::cout << "Button " << i << ": " << (int)input.buttons[i] << " ";
  }
  std::cout << std::endl;
}

int main(int argc, char* argv[]) {
  Viewer viewer;

  auto joysticks = viewer.GetListOfJoysticks();
  if (joysticks.empty()) {
    std::cout << "No joysticks found" << std::endl;
    return -1;
  }

  std::cout << "Available Joysticks:" << std::endl;
  for (int i = 0; i < joysticks.size(); ++i) {
    auto js = joysticks[i];
    std::cout << "[" << i << "] Joystick ID: " << js.id << ", Name: " << js.name
              << std::endl;
  }

  std::cout << "Please select which joystick to monitor: ";
  int selection;
  std::cin >> selection;
  if (selection < 0 || selection >= joysticks.size()) {
    std::cout << "Invalid joystick ID" << std::endl;
    return -1;
  }

  viewer.EnableJoystickInput(true);
  viewer.SetJoystickDeviceChangeCallback(JoystickDeviceChangeCallback);
  viewer.MonitorJoystickInputUpdate(joysticks[selection].id,
                                    JoystickInputUpdateCallback);

  auto panel = std::make_shared<ImGuiFixedPanel>("random-imgui_panel");
  viewer.AddSceneObject(panel);

  viewer.Show();
  return 0;
}
