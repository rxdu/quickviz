/*
 * @file panel.cpp
 * @date 9/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/panel.hpp"

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
  for (auto renderable : renderables_) {
    if (renderable->IsVisible()) renderable->OnRender();
  }
}
}  // namespace quickviz