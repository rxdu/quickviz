/*
 * @file layer.cpp
 * @date 9/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/gl_layer.hpp"

#include <iostream>

#include <yoga/Yoga.h>

namespace quickviz {
GlLayer::GlLayer(std::string name) : Layer(name) {}

void GlLayer::OnResize(float width, float height) {
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
        //    resizables_[i]->SetPosition(
        //        YGNodeLayoutGetLeft(child),
        //        height - YGNodeLayoutGetTop(child) -
        //        YGNodeLayoutGetHeight(child));
        resizables_[i]->OnResize(YGNodeLayoutGetWidth(child),
                             YGNodeLayoutGetHeight(child));
  }
}
}  // namespace quickviz