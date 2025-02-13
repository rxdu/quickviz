/*
 * @file test_joystick_input.cpp
 * @date 2/13/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>

#include "imview/panel.hpp"
#include "imview/viewer.hpp"

using namespace quickviz;

class JoystickPanel : public Panel {
 public:
  JoystickPanel(std::shared_ptr<Viewer> viewer)
      : Panel("JoystickPanel"), viewer_(viewer) {}

  void Draw() override {
    Begin();
    joysticks_ = viewer_->GetListOfJoysticks();
    ImGui::Text("Number of Joystick devices: %ld", joysticks_.size());

    if (!joysticks_.empty()) {
      static int selected_joystick_ = 0;
      if (ImGui::BeginCombo("Select Joystick",
                            joysticks_[selected_joystick_].name.c_str())) {
        for (int i = 0; i < joysticks_.size(); ++i) {
          auto js = joysticks_[i];
          std::string name = "[" + std::to_string(js.id) + "] " + js.name;
          if (ImGui::Selectable(name.c_str(), selected_joystick_ == i)) {
            selected_joystick_ = i;  // js.id;
          }
        }
        ImGui::EndCombo();
      }

      if (!viewer_->IsJoystickInputUpdateCallbackRegistered()) {
        if (ImGui::Button("Connect")) {
          std::cerr << "Monitor Joystick Input: " << "[" << selected_joystick_
                    << "] " << joysticks_[selected_joystick_].id << std::endl;
          viewer_->RegisterJoystickInputUpdateCallback(
              joysticks_[selected_joystick_].id,
              [this](const JoystickInput& input) {
                current_joystick_input_ = input;
                std::cerr << "Joystick Input Update: " << input.device.id
                          << std::endl;
              });
        }
      } else {
        if (ImGui::Button("Disconnect")) {
          viewer_->UnregisterJoystickInputUpdateCallback();
          current_joystick_input_.device.id = -1;
        }
      }

      // display current joystick input
      if (current_joystick_input_.device.id >= GLFW_JOYSTICK_1 &&
          current_joystick_input_.device.id < GLFW_JOYSTICK_LAST) {
        ImGui::Text("Current Joystick Input:");
        ImGui::Text("Device ID: %d", current_joystick_input_.device.id);
        ImGui::Text("Device Name: %s",
                    current_joystick_input_.device.name.c_str());
        ImGui::Text("Axes:");
        for (int i = 0; i < current_joystick_input_.axes.size(); ++i) {
          ImGui::Text(" - Axis %d: %.2f", i, current_joystick_input_.axes[i]);
        }
        ImGui::Text("Buttons:");
        for (int i = 0; i < current_joystick_input_.buttons.size(); ++i) {
          ImGui::Text(" - Button %d: %d", i,
                      current_joystick_input_.buttons[i]);
        }
      }
    } else {
      ImGui::Text("No joystick detected");
    }

    End();
  }

 private:
  std::shared_ptr<Viewer> viewer_;
  std::vector<JoystickDevice> joysticks_;
  JoystickInput current_joystick_input_;
};

int main(int argc, char* argv[]) {
  auto viewer = std::make_shared<Viewer>();

  viewer->EnableJoystickInput(true);

  auto panel = std::make_shared<JoystickPanel>(viewer);
  viewer->AddSceneObject(panel);

  viewer->Show();

  return 0;
}
