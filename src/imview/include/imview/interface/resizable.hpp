/*
 * @file resizable.hpp
 * @date 9/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_RESIZABLE_HPP
#define QUICKVIZ_RESIZABLE_HPP

#include "imview/styling.hpp"

namespace quickviz {
class Resizable {
 public:
  virtual void SetPosition(float x, float y) {};
  virtual void OnResize(float width, float height) = 0;

  virtual void SetAlignContent(Styling::AlignContent content) = 0;
  virtual void SetAlignItems(Styling::AlignItems items) = 0;
  virtual void SetAspectRatio(float aspect_ratio) = 0;
  virtual void SetDisplay(Styling::Display display) = 0;
  virtual void SetFlexBasis(float basis) = 0;
  virtual void SetFlexGrow(float grow) = 0;
  virtual void SetFlexShrink(float shrink) = 0;
  virtual void SetFlexDirection(Styling::FlexDirection direction) = 0;
  virtual void SetFlexWrap(Styling::FlexWrap wrap) = 0;
  virtual void SetGap(Styling::Edge edge, float gap) = 0;
  virtual void SetEdgeInset(Styling::Edge edge, float inset) = 0;
  virtual void SetJustifyContent(Styling::JustifyContent content) = 0;
  virtual void SetLayoutDirection(Styling::LayoutDirection direction) = 0;
  virtual void SetMargin(Styling::Edge edge, float margin) = 0;
  virtual void SetPadding(Styling::Edge edge, float padding) = 0;
  virtual void SetBorder(Styling::Edge edge, float border) = 0;
  virtual void SetPositionType(Styling::PositionType type) = 0;
  virtual void SetMinWidth(float width) = 0;
  virtual void SetMinHeight(float height) = 0;
  virtual void SetMaxWidth(float width) = 0;
  virtual void SetMaxHeight(float height) = 0;
};
}  // namespace quickviz

#endif  // QUICKVIZ_RESIZABLE_HPP
