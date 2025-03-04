/**
 * @file tui_panel.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-02-27
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/terminal/tui_panel.hpp"

#include <cmath>

#ifdef ENABLE_AUTO_LAYOUT
#include <yoga/Yoga.h>
#include "yoga_utils.hpp"

using namespace quickviz::YogaUtils;
#endif

namespace quickviz {
TuiPanel::TuiPanel(const std::string &title) : SceneObject(title) {}

TuiPanel::~TuiPanel() { delwin(window_); }

void TuiPanel::OnRender() {
  if (window_ == nullptr) return;

  werase(window_);
  Draw();
  if (has_border_) {
    box(window_, 0, 0);
  }
  if (show_title_) {
    mvwprintw(window_, 0, 2, " %s ", GetName().c_str());
  }
  wrefresh(window_);
}

void TuiPanel::OnResize(float width, float height) {
  // Round to nearest integer since ncurses uses integer coordinates
  width_ = static_cast<int>(std::round(width));
  height_ = static_cast<int>(std::round(height));

  // Adjust dimensions if we have a border to ensure content fits
  int effective_width = width_;
  int effective_height = height_;
  if (has_border_) {
    effective_width = std::max(2, effective_width);
    effective_height = std::max(2, effective_height);
  }

  if (window_ == nullptr) {
    window_ = newwin(effective_height, effective_width, y_, x_);
  } else {
    wresize(window_, effective_height, effective_width);
    mvwin(window_, y_, x_);
  }
}

void TuiPanel::SetTitleBar(bool value) { show_title_ = value; }

void TuiPanel::SetNoBorder(bool value) { has_border_ = !value; }
}  // namespace quickviz
