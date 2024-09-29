/*
 * @file layer.cpp
 * @date 9/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/layer.hpp"

#include <iostream>

namespace quickviz {
Layer::Layer(std::string name) : Renderable(), name_(name) {
  is_container_ = true;
  root_node_ = YGNodeNew();
}

Layer::~Layer() { YGNodeFreeRecursive(root_node_); }

void Layer::OnResize(int width, int height) {
  std::cout << name_ << "::Layer::CalculateLayout: " << width << " " << height
            << std::endl;
  YGNodeStyleSetWidth(root_node_, width);
  YGNodeStyleSetHeight(root_node_, height);
}

void Layer::AddPanel(Panel *panel) { panels_.push_back(panel); }

void Layer::OnRender() {
  if (!visible_) return;

  //  for (auto renderable : renderables_) {
  //    if (renderable->IsVisible()) renderable->OnRender();
  //  }
}
}  // namespace quickviz