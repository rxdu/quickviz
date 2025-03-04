/**
 * @file nc_text.cpp
 * @date 1/29/23
 * @brief
 *
 * @copyright Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "imview/terminal/tui_text.hpp"

#include <iostream>

namespace quickviz {
int TuiText::active_attributes = 0;

void TuiText::InitColor(Color bg) {
  if (has_colors()) {
    use_default_colors();
    start_color();
    for (int i = TuiText::Color::kDefault; i < TuiText::Color::kLastColor; ++i) {
      init_pair(i, i, bg);
    }
  }
}

void TuiText::SetBackgroundColor(TuiText::Color bg) {
  for (int i = TuiText::Color::kDefault; i < TuiText::Color::kLastColor; ++i) {
    init_pair(i, i, bg);
  }
}

void TuiText::ResetBackgroundColor() {
  for (int i = TuiText::Color::kDefault; i < TuiText::Color::kLastColor; ++i) {
    init_pair(i, i, -1);
  }
}

void TuiText::SetAttribute(WINDOW *win, int attr) {
  active_attributes = attr;
  wattron(win, active_attributes);
}

void TuiText::ResetAttribute(WINDOW *win) { wattroff(win, active_attributes); }
}  // namespace quickviz