/*
 * @file console_panel.cpp
 * @date 10/8/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "panels/console_panel.hpp"

#include "imview/fonts.hpp"

namespace quickviz {
ConsolePanel::ConsolePanel(std::string name) : Panel(name) {
  this->SetAutoLayout(false);
  this->SetNoResize(true);
  this->SetNoMove(true);
  this->SetWindowNoMenuButton();

  static int counter = 0;
  const char* categories[3] = {"info", "warn", "error"};
  const char* words[] = {"Bumfuzzled",    "Cattywampus",  "Snickersnee",
                         "Abibliophobia", "Absquatulate", "Nincompoop",
                         "Pauciloquent"};
  for (int n = 0; n < 5; n++) {
    const char* category = categories[counter % IM_ARRAYSIZE(categories)];
    const char* word = words[counter % IM_ARRAYSIZE(words)];
    log_.AddLog(
        "[%05d] [%s] Hello, current time is %.1f, here's a word: '%s'\n",
        ImGui::GetFrameCount(), category, ImGui::GetTime(), word);
    counter++;
  }
}

void ConsolePanel::Draw() {
  Begin();
  log_.Draw("Example: Log");
  End();
}
}  // namespace quickviz