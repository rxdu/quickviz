/*
 * test_viewer.cpp
 *
 * Created on: Jul 27, 2021 09:07
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <iostream>

#include "imview/viewer.hpp"
#include "imview/box.hpp"

#include "imview/panel.hpp"
#include "imview/popup.hpp"

#include "imview/component/popup_manager.hpp"

using namespace quickviz;

class MyPanel : public Panel {
 public:
  MyPanel(std::string name = "MyPanel") : Panel(name) {
    this->SetAutoLayout(false);

    PopupManager::GetInstance().RegisterNotificationPopup(
        "Notification", "This is a popup test");
    PopupManager::GetInstance().RegisterConfirmationPopup(
        "Confirmation", "This is a confirmation test", [](bool confirm) {
          if (confirm) {
            std::cout << "Confirmed" << std::endl;
          } else {
            std::cout << "Not Confirmed" << std::endl;
          }
        });
  }

  void Draw() override {
    Begin();
    {
      static bool show_popup = false;
      if (ImGui::Button("Show Notification")) {
        PopupManager::GetInstance().TriggerPopup("Notification");
      }
      if (ImGui::Button("Show Confirmation")) {
        PopupManager::GetInstance().TriggerPopup("Confirmation");
      }

      PopupManager::GetInstance().UpdatePopups();

      ImGui::PushFont(Fonts::GetFont(FontSize::kFont18));
      ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 153, 153, 200));
      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                  1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
      ImGui::PopStyleColor();
      ImGui::PopFont();
    }
    End();
  }
};

int main(int argc, char* argv[]) {
  Viewer viewer;

  auto obj1 = std::make_shared<MyPanel>("Panel1");
  viewer.AddSceneObject(obj1);

  viewer.Show();
  return 0;
}