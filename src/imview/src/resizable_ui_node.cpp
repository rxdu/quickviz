/*
 * @file resizable_node.cpp
 * @date 9/30/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/details/resizable_ui_node.hpp"

#include "yoga_utils.hpp"

namespace quickviz {
using namespace YogaUtils;

ResizableUiNode::ResizableUiNode(bool is_container)
    : is_container_(is_container) {
  yg_node_ = YGNodeNew();
}

ResizableUiNode::~ResizableUiNode() { YGNodeFreeRecursive(yg_node_); }

void ResizableUiNode::SetAlignContent(Styling::AlignContent align) {
  YGNodeStyleSetAlignContent(yg_node_, ToYogaAlign(align));
}

void ResizableUiNode::SetAlignItems(Styling::AlignItems items) {
  YGNodeStyleSetAlignItems(yg_node_, ToYogaAlign(items));
}

void ResizableUiNode::SetAspectRatio(float aspect_ratio) {
  YGNodeStyleSetAspectRatio(yg_node_, aspect_ratio);
}

void ResizableUiNode::SetDisplay(Styling::Display display) {
  YGNodeStyleSetDisplay(yg_node_, ToYogaDisplay(display));
}

void ResizableUiNode::SetFlexBasis(float basis) {
  YGNodeStyleSetFlexBasis(yg_node_, basis);
}

void ResizableUiNode::SetFlexGrow(float grow) {
  YGNodeStyleSetFlexGrow(yg_node_, grow);
}

void ResizableUiNode::SetFlexShrink(float shrink) {
  YGNodeStyleSetFlexShrink(yg_node_, shrink);
}

void ResizableUiNode::SetFlexDirection(Styling::FlexDirection direction) {
  YGNodeStyleSetFlexDirection(yg_node_, ToYogaFlexDirection(direction));
}

void ResizableUiNode::SetFlexWrap(Styling::FlexWrap wrap) {
  YGNodeStyleSetFlexWrap(yg_node_, ToYogaFlexWrap(wrap));
}

void ResizableUiNode::SetGap(Styling::Edge edge, float gap) {
  YGNodeStyleSetMargin(yg_node_, ToYogaEdge(edge), gap);
}

void ResizableUiNode::SetEdgeInset(Styling::Edge edge, float inset) {
  YGNodeStyleSetPadding(yg_node_, ToYogaEdge(edge), inset);
}

void ResizableUiNode::SetJustifyContent(Styling::JustifyContent content) {
  YGNodeStyleSetJustifyContent(yg_node_, ToYogaJustify(content));
}

void ResizableUiNode::SetLayoutDirection(Styling::LayoutDirection direction) {
  YGNodeStyleSetDirection(yg_node_, ToYogaDirection(direction));
}

void ResizableUiNode::SetMargin(Styling::Edge edge, float margin) {
  YGNodeStyleSetMargin(yg_node_, ToYogaEdge(edge), margin);
}

void ResizableUiNode::SetPadding(Styling::Edge edge, float padding) {
  YGNodeStyleSetPadding(yg_node_, ToYogaEdge(edge), padding);
}

void ResizableUiNode::SetBorder(Styling::Edge edge, float border) {
  YGNodeStyleSetBorder(yg_node_, ToYogaEdge(edge), border);
}

void ResizableUiNode::SetPositionType(Styling::PositionType type) {
  YGNodeStyleSetPositionType(yg_node_, ToYogaPositionType(type));
}

void ResizableUiNode::SetMinWidth(float width) {
  YGNodeStyleSetMinWidth(yg_node_, width);
}

void ResizableUiNode::SetMinHeight(float height) {
  YGNodeStyleSetMinHeight(yg_node_, height);
}

void ResizableUiNode::SetMaxWidth(float width) {
  YGNodeStyleSetMaxWidth(yg_node_, width);
}

void ResizableUiNode::SetMaxHeight(float height) {
  YGNodeStyleSetMaxHeight(yg_node_, height);
}
}  // namespace quickviz