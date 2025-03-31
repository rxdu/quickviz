/*
 * @file popup_manager.cpp
 * @date 12/18/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/popup_manager.hpp"

#include "imgui/imgui.h"

namespace quickviz {
void PopupManager::RegisterNotificationPopup(std::string title, std::string msg,
                                             float width, float height) {
  std::lock_guard<std::mutex> lock(popup_mtx_);
  popups_[title].triggered = false;
  popups_[title].descriptor = PopupDescriptor{
      PopupType::kNotification, title, msg, width, height, nullptr};
}

void PopupManager::RegisterConfirmationPopup(std::string title, std::string msg,
                                             OnConfirmationCallback callback,
                                             float width, float height) {
  std::lock_guard<std::mutex> lock(popup_mtx_);
  popups_[title].triggered = false;
  popups_[title].descriptor = PopupDescriptor{
      PopupType::kConfirmation, title, msg, width, height, callback};
}

void PopupManager::TriggerPopup(std::string title) {
  std::lock_guard<std::mutex> lock(popup_mtx_);
  if (popups_.find(title) != popups_.end()) {
    popups_[title].triggered = true;
  }
}

void PopupManager::CancelPopup(std::string title) {
  std::lock_guard<std::mutex> lock(popup_mtx_);
  if (popups_.find(title) != popups_.end()) {
    popups_[title].triggered = false;
  }
}

void PopupManager::UpdatePopups() {
  std::unordered_map<std::string, PopupState> popups;

  // update popup date
  {
    std::lock_guard<std::mutex> lock(popup_mtx_);
    popups = popups_;
    for (auto& [title, state] : popups_) {
      if (state.triggered) {
        ImGui::OpenPopup(title.c_str());
        state.triggered = false;
      }
    }
  }

  // show popups if triggered
  for (auto& [title, state] : popups) {
    switch (state.descriptor.type) {
      case PopupType::kNotification:
        ShowNotificationPopup(state.descriptor.title, state.descriptor.msg,
                              state.descriptor.width, state.descriptor.height);
        break;
      case PopupType::kConfirmation:
        ShowConfirmationPopup(state.descriptor.title, state.descriptor.msg,
                              state.descriptor.callback, state.descriptor.width,
                              state.descriptor.height);
        break;
    }
  }
}
}  // namespace quickviz