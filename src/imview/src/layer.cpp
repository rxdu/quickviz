/*
 * @file layer.cpp
 * @date 9/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/layer.hpp"

#include <iostream>

#ifdef ENABLE_AUTO_LAYOUT
#include <yoga/Yoga.h>
#endif

namespace quickviz {
Layer::Layer(std::string name) : SceneObject(name) {}

void Layer::AddChild(std::shared_ptr<SceneObject> obj) {
  if (obj == nullptr) return;
  children_[obj->GetName()] = obj;
#ifdef ENABLE_AUTO_LAYOUT
  auto idx = YGNodeGetChildCount(yg_node_);
  child_name_by_index_[idx] = obj->GetName();
  YGNodeInsertChild(yg_node_, obj->GetYogaNode(), idx);
#endif
}

void Layer::RemoveChild(const std::string& name) {
  for (auto& child : child_name_by_index_) {
    if (child.second == name) {
      child_name_by_index_.erase(child.first);
      break;
    }
  }
  auto it = children_.find(name);
  if (it != children_.end()) {
#ifdef ENABLE_AUTO_LAYOUT
    YGNodeRemoveChild(yg_node_, it->second->GetYogaNode());
#endif
    children_.erase(it);
  }
}

void Layer::OnResize(float width, float height) {
  for (auto child : children_) {
    child.second->OnResize(width, height);
  }
}

void Layer::OnRender() {
  if (!visible_) return;

  //  std::cout << "Rendering Layer: " << name_ << std::endl;
  for (auto child : children_) {
    if (child.second->IsVisible()) child.second->OnRender();
  }
}
}  // namespace quickviz