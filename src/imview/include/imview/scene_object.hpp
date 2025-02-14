/*
 * @file scene_object.hpp
 * @date 9/30/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_SCENE_OBJECT_HPP
#define QUICKVIZ_SCENE_OBJECT_HPP

#include <cstddef>
#include <string>

#include "imview/interface/resizable.hpp"
#include "imview/interface/renderable.hpp"
#include "imview/interface/input_handler.hpp"

#ifdef ENABLE_AUTO_LAYOUT
struct YGNode;
typedef struct YGNode* YGNodeRef;
#endif

namespace quickviz {
class SceneObject : public Resizable, public Renderable, public InputHandler {
 public:
  SceneObject(std::string name);
  virtual ~SceneObject();

  /****** public methods ******/
  // functions to be (re)implemented by derived classes
  virtual void SetPosition(float x, float y);
  virtual void OnResize(float width, float height);
  virtual void OnRender() = 0;

  // common methods
  std::string GetName() const { return name_; }
  void SetVisibility(bool visible) { visible_ = visible; }
  bool IsVisible() const override { return visible_; }

  // used for automatic layout only
#ifdef ENABLE_AUTO_LAYOUT
  YGNodeRef GetYogaNode() { return yg_node_; }
  void SetAlignContent(Styling::AlignContent content) override;
  void SetAlignItems(Styling::AlignItems alignment) override;
  void SetAlignSelf(Styling::AlignSelf alignment) override;
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
  void SetHeight(float height) override;
  void SetWidth(float width) override;
  void SetHeightPercent(float height) override;
  void SetWidthPercent(float width) override;
  void SetMinWidth(float width) override;
  void SetMinHeight(float height) override;
  void SetMaxWidth(float width) override;
  void SetMaxHeight(float height) override;
#endif

  // user input handling
  void SetInputHandlingStrategy(Type type, Strategy strategy) override;
  void OnJoystickDeviceChange(
      const std::vector<JoystickDevice>& devices) override;

  virtual void OnJoystickUpdate(const JoystickInput& input) = 0;

 protected:
  std::string name_;
  bool visible_ = true;

  float x_ = 0;
  float y_ = 0;
  float width_ = 0;
  float height_ = 0;

#ifdef ENABLE_AUTO_LAYOUT
  YGNodeRef yg_node_;
  size_t child_count_ = 0;
#endif

  std::vector<JoystickDevice> joysticks_;
  std::unordered_map<InputHandler::Type, InputHandler::Strategy>
      input_handling_strategies_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_SCENE_OBJECT_HPP