/**
 * @file tui_panel.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-02-27
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/terminal/tui_panel.hpp"

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
  if (has_border_) {
    box(window_, 0, 0);
  }
  Draw();
  wrefresh(window_);
}

void TuiPanel::OnResize(float width, float height) {
  width_ = width;
  height_ = height;

  if (window_ == nullptr) {
    window_ = newwin(height_, width_, y_, x_);
  } else {
    wresize(window_, height_, width_);
    mvwin(window_, y_, x_);
  }
}

void TuiPanel::SetNoBorder(bool value) { has_border_ = !value; }
}  // namespace quickviz
