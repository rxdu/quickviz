/*
 * @file panel.cpp
 * @date 9/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/panel.hpp"

#include <iostream>

#include "glad/glad.h"

namespace quickviz {
Panel::Panel(std::string name) : ResizableUiNode(true), name_(name) {}

Panel::~Panel() {}

void Panel::AddRenderable(std::shared_ptr<Renderable> renderable) {
  //  YGNodeInsertChild(yg_node_, child1, 0);
  renderables_.push_back(renderable);
}

void Panel::SetPosition(float x, float y) {
  x_ = x;
  y_ = y;
}

void Panel::OnResize(float width, float height) {
  width_ = width;
  height_ = height;
}

void Panel::OnRender() {
  if (!visible_) return;
  glEnable(GL_SCISSOR_TEST);
  glViewport(x_, y_, width_, height_);
  glScissor(x_, y_, width_, height_);
  glClearColor(bg_color_[0], bg_color_[1], bg_color_[2], bg_color_[3]);
  glClear(GL_COLOR_BUFFER_BIT);
  for (auto renderable : renderables_) {
    if (renderable->IsVisible()) renderable->OnRender();
  }
  glDisable(GL_SCISSOR_TEST);
}
}  // namespace quickviz