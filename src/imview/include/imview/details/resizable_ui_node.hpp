/*
 * @file resizable_ui_node.hpp
 * @date 9/30/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_RESIZABLE_UI_NODE_HPP
#define QUICKVIZ_RESIZABLE_UI_NODE_HPP

#include <yoga/Yoga.h>

#include "imview/interface/resizable.hpp"
#include "imview/interface/renderable.hpp"

namespace quickviz {
class ResizableUiNode : public Resizable, public Renderable {
 public:
  ResizableUiNode(bool is_container = false);
  virtual ~ResizableUiNode();

  // public methods
  void SetVisible(bool visible) { visible_ = visible; }
  bool IsVisible() const override { return visible_; }
  bool IsContainer() const { return is_container_; }
  YGNodeRef GetYogaNode() { return yg_node_; }

  void SetAlignContent(Styling::AlignContent content) override;
  void SetAlignItems(Styling::AlignItems items) override;
  void SetAspectRatio(float aspect_ratio) override;
  void SetDisplay(Styling::Display display) override;
  void SetFlexBasis(float basis) override;
  void SetFlexGrow(float grow) override;
  void SetFlexShrink(float shrink) override;
  void SetFlexDirection(Styling::FlexDirection direction) override;
  void SetFlexWrap(Styling::FlexWrap wrap) override;
  void SetGap(Styling::Edge edge, float gap) override;
  void SetEdgeInset(Styling::Edge edge, float inset) override;
  void SetJustifyContent(Styling::JustifyContent content) override;
  void SetLayoutDirection(Styling::LayoutDirection direction) override;
  void SetMargin(Styling::Edge edge, float margin) override;
  void SetPadding(Styling::Edge edge, float padding) override;
  void SetBorder(Styling::Edge edge, float border) override;
  void SetPositionType(Styling::PositionType type) override;
  void SetMinWidth(float width) override;
  void SetMinHeight(float height) override;
  void SetMaxWidth(float width) override;
  void SetMaxHeight(float height) override;

  // to be implemented by derived classes
  virtual void OnResize(float width, float height) = 0;
  virtual void OnRender() = 0;

 protected:
  YGNodeRef yg_node_;
  size_t child_count_ = 0;
  bool visible_ = true;
  bool is_container_ = false;
};
}  // namespace quickviz

#endif  // QUICKVIZ_RESIZABLE_UI_NODE_HPP