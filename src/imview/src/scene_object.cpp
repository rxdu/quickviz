/*
 * @file scene_object.cpp
 * @date 9/30/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/scene_object.hpp"

#include <iostream>

#ifdef ENABLE_AUTO_LAYOUT
#include <yoga/Yoga.h>
#include "yoga_utils.hpp"

using namespace quickviz::YogaUtils;
#endif

namespace quickviz {
SceneObject::SceneObject(std::string name) : name_(std::move(name)) {
#ifdef ENABLE_AUTO_LAYOUT
  yg_node_ = YGNodeNew();
#endif

  input_handling_strategies_[InputHandler::Type::kKeyboard] =
      InputHandler::Strategy::kNone;
  input_handling_strategies_[InputHandler::Type::kMouse] =
      InputHandler::Strategy::kNone;
  input_handling_strategies_[InputHandler::Type::kJoystick] =
      InputHandler::Strategy::kNone;
}

SceneObject::~SceneObject() {
#ifdef ENABLE_AUTO_LAYOUT
  YGNodeFreeRecursive(yg_node_);
#endif
}

void SceneObject::SetPosition(float x, float y) {
  x_ = x;
  y_ = y;
}

void SceneObject::OnResize(float width, float height) {
  width_ = width;
  height_ = height;
}

#ifdef ENABLE_AUTO_LAYOUT
void SceneObject::SetAlignContent(Styling::AlignContent align) {
  YGNodeStyleSetAlignContent(yg_node_, ToYogaAlign(align));
}

void SceneObject::SetAlignItems(Styling::AlignItems alignment) {
  YGNodeStyleSetAlignItems(yg_node_, ToYogaAlign(alignment));
}

void SceneObject::SetAlignSelf(Styling::AlignSelf alignment) {
  YGNodeStyleSetAlignSelf(yg_node_, ToYogaAlign(alignment));
}

void SceneObject::SetAspectRatio(float aspect_ratio) {
  YGNodeStyleSetAspectRatio(yg_node_, aspect_ratio);
}

void SceneObject::SetDisplay(Styling::Display display) {
  YGNodeStyleSetDisplay(yg_node_, ToYogaDisplay(display));
}

void SceneObject::SetFlexBasis(float basis) {
  YGNodeStyleSetFlexBasis(yg_node_, basis);
}

void SceneObject::SetFlexGrow(float grow) {
  YGNodeStyleSetFlexGrow(yg_node_, grow);
}

void SceneObject::SetFlexShrink(float shrink) {
  YGNodeStyleSetFlexShrink(yg_node_, shrink);
}

void SceneObject::SetFlexDirection(Styling::FlexDirection direction) {
  YGNodeStyleSetFlexDirection(yg_node_, ToYogaFlexDirection(direction));
}

void SceneObject::SetFlexWrap(Styling::FlexWrap wrap) {
  YGNodeStyleSetFlexWrap(yg_node_, ToYogaFlexWrap(wrap));
}

void SceneObject::SetGap(Styling::Edge edge, float gap) {
  YGNodeStyleSetMargin(yg_node_, ToYogaEdge(edge), gap);
}

void SceneObject::SetEdgeInset(Styling::Edge edge, float inset) {
  YGNodeStyleSetPadding(yg_node_, ToYogaEdge(edge), inset);
}

void SceneObject::SetJustifyContent(Styling::JustifyContent content) {
  YGNodeStyleSetJustifyContent(yg_node_, ToYogaJustify(content));
}

void SceneObject::SetLayoutDirection(Styling::LayoutDirection direction) {
  YGNodeStyleSetDirection(yg_node_, ToYogaDirection(direction));
}

void SceneObject::SetMargin(Styling::Edge edge, float margin) {
  YGNodeStyleSetMargin(yg_node_, ToYogaEdge(edge), margin);
}

void SceneObject::SetPadding(Styling::Edge edge, float padding) {
  YGNodeStyleSetPadding(yg_node_, ToYogaEdge(edge), padding);
}

void SceneObject::SetBorder(Styling::Edge edge, float border) {
  YGNodeStyleSetBorder(yg_node_, ToYogaEdge(edge), border);
}

void SceneObject::SetPositionType(Styling::PositionType type) {
  YGNodeStyleSetPositionType(yg_node_, ToYogaPositionType(type));
}

void SceneObject::SetHeight(float height) {
  YGNodeStyleSetHeight(yg_node_, height);
}

void SceneObject::SetWidth(float width) {
  YGNodeStyleSetWidth(yg_node_, width);
}

void SceneObject::SetHeightPercent(float height) {
  YGNodeStyleSetHeightPercent(yg_node_, height);
}

void SceneObject::SetWidthPercent(float width) {
  YGNodeStyleSetWidthPercent(yg_node_, width);
}

void SceneObject::SetMinWidth(float width) {
  YGNodeStyleSetMinWidth(yg_node_, width);
}

void SceneObject::SetMinHeight(float height) {
  YGNodeStyleSetMinHeight(yg_node_, height);
}

void SceneObject::SetMaxWidth(float width) {
  YGNodeStyleSetMaxWidth(yg_node_, width);
}

void SceneObject::SetMaxHeight(float height) {
  YGNodeStyleSetMaxHeight(yg_node_, height);
}
#endif

void SceneObject::SetInputHandlingStrategy(Type type, Strategy strategy) {
  if (input_handling_strategies_.find(type) == input_handling_strategies_.end())
    return;
  input_handling_strategies_[type] = strategy;
}

void SceneObject::OnJoystickDeviceChange(
    const std::vector<JoystickDevice>& devices) {
  joysticks_ = devices;
  // std::cout << "Joystick device changed, available joystick number: "
  //           << devices.size() << std::endl;
  // std::cout << " - Current scene object: " << name_ << std::endl;
  // for (int i = 0; i < joysticks_.size(); ++i) {
  //   std::cout << " - Joystick " << i << ": " << joysticks_[i].id << " - "
  //             << joysticks_[i].name << std::endl;
  // }
}
}  // namespace quickviz