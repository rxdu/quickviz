/*
 * @file box.cpp
 * @date 10/6/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/box.hpp"

#include <iostream>

#include <yoga/Yoga.h>

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

Box::Box(std::string name) : SceneObject(name) {}

void Box::PrintLayout() const {
  std::cout << "Box: " << name_ << std::endl;
  PrintYgLayout(yg_node_);
}

void Box::AddChild(std::shared_ptr<SceneObject> obj) {
  if (obj == nullptr) return;
  children_[obj->GetName()] = obj;
  auto idx = YGNodeGetChildCount(yg_node_);
  child_name_by_index_[idx] = obj->GetName();
  YGNodeInsertChild(yg_node_, obj->GetYogaNode(), idx);
}

void Box::RemoveChild(const std::string& name) {
  for (auto& child : child_name_by_index_) {
    if (child.second == name) {
      child_name_by_index_.erase(child.first);
      break;
    }
  }
  auto it = children_.find(name);
  if (it != children_.end()) {
    YGNodeRemoveChild(yg_node_, it->second->GetYogaNode());
    children_.erase(it);
  }
}

void Box::OnResize(float width, float height) {
  // update root node size
  YGNodeStyleSetWidth(yg_node_, width);
  YGNodeStyleSetHeight(yg_node_, height);
  YGNodeCalculateLayout(yg_node_, YGUndefined, YGUndefined, YGDirectionLTR);
  for (auto child : children_) {
    child.second->OnResize(width, height);
  }

  for (uint32_t i = 0; i < YGNodeGetChildCount(yg_node_); ++i) {
    auto child = YGNodeGetChild(yg_node_, i);
    children_[child_name_by_index_[i]]->SetPosition(YGNodeLayoutGetLeft(child),
                                                    YGNodeLayoutGetTop(child));
    children_[child_name_by_index_[i]]->OnResize(YGNodeLayoutGetWidth(child),
                                                 YGNodeLayoutGetHeight(child));
  }

  PrintLayout();
}

void Box::OnRender() {
  if (!visible_) return;

  //  std::cout << "Rendering Box: " << name_ << std::endl;
  for (auto child : children_) {
    if (child.second->IsVisible()) child.second->OnRender();
  }
}
}  // namespace quickviz