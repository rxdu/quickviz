/**
 * @file tui_panel.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-02-27
 * @brief 
 * 
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <yoga/yoga.h>

#include "ncview/nc_types.hpp"

namespace xmotion {
namespace swviz {

class TuiPanel {
 public:
  TuiPanel(const std::string& title = "");
  virtual ~TuiPanel();

  // Panel properties
  void SetTitle(const std::string& title) { title_ = title; }
  const std::string& GetTitle() const { return title_; }

  void SetBorder(bool has_border) { has_border_ = has_border; }
  bool HasBorder() const { return has_border_; }

  // Layout control
  void SetPosition(int x, int y);
  void SetSize(int width, int height);
  void SetAutoResize(bool enable) { auto_resize_ = enable; }

  // Flexbox properties
  void SetFlexDirection(YGFlexDirection direction);
  void SetJustifyContent(YGJustify justify);
  void SetAlignItems(YGAlign align);
  void SetFlexGrow(float flex_grow);
  void SetFlexShrink(float flex_shrink);
  void SetFlexBasis(float flex_basis);
  void SetMargin(YGEdge edge, float margin);
  void SetPadding(YGEdge edge, float padding);

  // Child management
  void AddPanel(std::shared_ptr<TuiPanel> panel);
  void RemovePanel(std::shared_ptr<TuiPanel> panel);
  void ClearPanels();

  // Drawing interface
  virtual void OnResize(int rows, int cols, int y, int x);
  virtual void OnDraw();

  // Region access for child panels
  int GetX() const { return region_.x; }
  int GetY() const { return region_.y; }
  int GetWidth() const { return region_.width; }
  int GetHeight() const { return region_.height; }

 protected:
  std::string title_;
  bool has_border_ = true;
  bool auto_resize_ = true;

  struct {
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
  } region_;

  std::vector<std::shared_ptr<TuiPanel>> child_panels_;
  YGNodeRef yoga_node_;

  void CalculateLayout();
  void ApplyLayout();
  void CalcContentRegion();
};

} // namespace swviz
} // namespace xmotion 