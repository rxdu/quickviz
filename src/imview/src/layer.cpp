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
namespace {
void PrintYgLayout(YGNodeRef node, int indent = 0) {
  for (int i = 0; i < indent; ++i) std::cout << "  ";
  std::cout << "Size: "
            << "X: " << YGNodeLayoutGetLeft(node)
            << ", Y: " << YGNodeLayoutGetTop(node)
            << ", Width: " << YGNodeLayoutGetWidth(node)
            << ", Height: " << YGNodeLayoutGetHeight(node) << std::endl;

  // Recursively print children nodes
  for (uint32_t i = 0; i < YGNodeGetChildCount(node); ++i) {
    PrintYgLayout(YGNodeGetChild(node, i), indent + 1);
  }
}
}  // namespace

Layer::Layer(std::string name) : ResizableUiNode(true), name_(name) {}

void Layer::PrintLayout() const {
  std::cout << "Layer: " << name_ << std::endl;
  PrintYgLayout(yg_node_);
}

void Layer::AddResizableUiNode(std::shared_ptr<ResizableUiNode> resizable) {
  int idx = child_count_++;
  resizables_[idx] = resizable;
  YGNodeInsertChild(yg_node_, resizable->GetYogaNode(), idx);
}

void Layer::AddRenderable(std::shared_ptr<Renderable> renderable) {
  renderables_.push_back(renderable);
}

void Layer::OnResize(float width, float height) {
  // update root node size
  YGNodeStyleSetWidth(yg_node_, width);
  YGNodeStyleSetHeight(yg_node_, height);
  YGNodeCalculateLayout(yg_node_, YGUndefined, YGUndefined, YGDirectionLTR);
  for (auto resizable : resizables_) {
    resizable.second->OnResize(width, height);
  }

  for (uint32_t i = 0; i < YGNodeGetChildCount(yg_node_); ++i) {
    auto child = YGNodeGetChild(yg_node_, i);
    resizables_[i]->SetPosition(YGNodeLayoutGetLeft(child),
                                YGNodeLayoutGetTop(child));
    resizables_[i]->OnResize(YGNodeLayoutGetWidth(child),
                             YGNodeLayoutGetHeight(child));
  }
}

void Layer::OnRender() {
  if (!visible_) return;

  for (auto resizable : resizables_) {
    if (resizable.second->IsVisible()) resizable.second->OnRender();
  }

  for (auto renderable : renderables_) {
    if (renderable->IsVisible()) renderable->OnRender();
  }
}
}  // namespace quickviz