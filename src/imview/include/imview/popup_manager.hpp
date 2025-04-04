/*
 * @file popup_manager.hpp
 * @date 12/18/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef POPUP_MANAGER_HPP
#define POPUP_MANAGER_HPP

#include <mutex>

#include "imview/popup.hpp"

namespace quickviz {
class PopupManager {
  struct PopupDescriptor {
    PopupType type;
    std::string title;
    std::string msg;
    float width;
    float height;
    OnConfirmationCallback callback;
  };

  struct PopupState {
    bool triggered;
    PopupDescriptor descriptor;
  };

 public:
  static PopupManager& GetInstance() {
    static PopupManager instance;
    return instance;
  }

  void RegisterNotificationPopup(std::string title, std::string msg,
                                 float width = 300, float height = 150);
  void RegisterConfirmationPopup(std::string title, std::string msg,
                                 OnConfirmationCallback callback = nullptr,
                                 float width = 300, float height = 150);

  void TriggerPopup(std::string title);
  void CancelPopup(std::string title);
  void UpdatePopups();

 private:
  PopupManager() = default;
  ~PopupManager() = default;

  std::mutex popup_mtx_;
  std::unordered_map<std::string, PopupState> popups_;
};
}  // namespace quickviz

#endif  // POPUP_MANAGER_HPP