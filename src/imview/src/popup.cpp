/*
 * popup.cpp
 *
 * Created on 4/5/22 11:08 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#include "imview/popup.hpp"

#include "imgui.h"

namespace quickviz {
void ShowNotificationPopup(std::string title, std::string msg, float width,
                           float height) {
  // Always center this window when appearing
  ImVec2 center = ImGui::GetMainViewport()->GetCenter();
  ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  ImGui::SetNextWindowSize(ImVec2(width, height));
  ImGui::SetNextWindowBgAlpha(0.75f);

  if (ImGui::BeginPopupModal(
          title.c_str(), NULL,
          ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoResize)) {
    ImGui::SetCursorPos(ImVec2(15, 40));
    ImGui::Text("%s", msg.c_str());
    ImGui::Text("\n");
    // ImGui::Separator();

    ImGui::SetItemDefaultFocus();
    ImGui::SetCursorPos(ImVec2((width - 120) / 2.0f, 100));
    if (ImGui::Button("OK", ImVec2(120, 0))) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::SetItemDefaultFocus();
    ImGui::EndPopup();
  }
}

void ShowConfirmationPopup(std::string title, std::string msg,
                           OnConfirmationCallback callback, float width,
                           float height) {
  // Always center this window when appearing
  ImVec2 center = ImGui::GetMainViewport()->GetCenter();
  ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  ImGui::SetNextWindowSize(ImVec2(width, height));
  ImGui::SetNextWindowBgAlpha(0.75f);

  if (ImGui::BeginPopupModal(
          title.c_str(), NULL,
          ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoResize)) {
    ImGui::SetCursorPos(ImVec2(15, 40));
    ImGui::Text("%s", msg.c_str());
    ImGui::Text("\n");
    // ImGui::Separator();

    ImGui::SetItemDefaultFocus();
    ImGui::SetCursorPos(ImVec2((width - 170) / 2.0f, 100));
    if (ImGui::Button("Yes", ImVec2(80, 0))) {
      callback(true);
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("No", ImVec2(80, 0))) {
      callback(false);
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}
}  // namespace quickviz