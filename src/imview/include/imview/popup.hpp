/*
 * popup.hpp
 *
 * Created on 4/5/22 11:08 PM
 * Description:
 *
 * Copyright (c) 2022 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_SRC_VISUALIZATION_IMVIEW_INCLUDE_IMVIEW_POPUP_HPP
#define ROBOSW_SRC_VISUALIZATION_IMVIEW_INCLUDE_IMVIEW_POPUP_HPP

#include <string>
#include <cstdint>
#include <functional>

namespace quickviz {
enum class PopupType { kNotification, kConfirmation };

void ShowNotificationPopup(std::string title, std::string msg,
                           float width = 300, float height = 150);

using OnConfirmationCallback = std::function<void(bool)>;
void ShowConfirmationPopup(std::string title, std::string msg,
                           OnConfirmationCallback callback = nullptr,
                           float width = 300, float height = 150);
}  // namespace quickviz

#endif  // ROBOSW_SRC_VISUALIZATION_IMVIEW_INCLUDE_IMVIEW_POPUP_HPP
