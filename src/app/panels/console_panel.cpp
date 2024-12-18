/*
 * @file console_panel.cpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "panels/console_panel.hpp"

#include "imview/fonts.hpp"
#include "imview/component/logging/app_log_handler.hpp"

namespace quickviz {
ConsolePanel::ConsolePanel(std::string name) : Panel(name) {
  this->SetAutoLayout(false);
  this->SetNoResize(true);
  this->SetNoMove(true);
  this->SetWindowNoMenuButton();

  AppLogHandler::GetInstance().Log(LogLevel::kInfo, "app initialized");
}

void ConsolePanel::Draw() {
  Begin();
  // log_.Draw("Example: Log");
  AppLogHandler::GetInstance().Draw();
  End();
}
}  // namespace quickviz