#include "ncview/panel.hpp"

#include <ncurses.h>
#include <algorithm>
#include <yoga/Yoga.h>

namespace xmotion {
namespace swviz {

Panel::Panel(const std::string& title) : title_(title) {
  // Initialize Yoga node
  yoga_node_ = YGNodeNew();
  YGNodeStyleSetFlexDirection(yoga_node_, YGFlexDirectionColumn);
  YGNodeStyleSetWidth(yoga_node_, YGUndefined);
  YGNodeStyleSetHeight(yoga_node_, YGUndefined);
}

Panel::~Panel() {
  if (yoga_node_) {
    YGNodeFree(yoga_node_);
  }
}

void Panel::SetPosition(int x, int y) {
  region_.x = x;
  region_.y = y;
}

void Panel::SetSize(int width, int height) {
  region_.width = width;
  region_.height = height;
  
  YGNodeStyleSetWidth(yoga_node_, width);
  YGNodeStyleSetHeight(yoga_node_, height);
}

// Flexbox property setters
void Panel::SetFlexDirection(YGFlexDirection direction) {
  YGNodeStyleSetFlexDirection(yoga_node_, direction);
}

void Panel::SetJustifyContent(YGJustify justify) {
  YGNodeStyleSetJustifyContent(yoga_node_, justify);
}

void Panel::SetAlignItems(YGAlign align) {
  YGNodeStyleSetAlignItems(yoga_node_, align);
}

void Panel::SetFlexGrow(float flex_grow) {
  YGNodeStyleSetFlexGrow(yoga_node_, flex_grow);
}

void Panel::SetFlexShrink(float flex_shrink) {
  YGNodeStyleSetFlexShrink(yoga_node_, flex_shrink);
}

void Panel::SetFlexBasis(float flex_basis) {
  YGNodeStyleSetFlexBasis(yoga_node_, flex_basis);
}

void Panel::SetMargin(YGEdge edge, float margin) {
  YGNodeStyleSetMargin(yoga_node_, edge, margin);
}

void Panel::SetPadding(YGEdge edge, float padding) {
  YGNodeStyleSetPadding(yoga_node_, edge, padding);
}

void Panel::AddPanel(std::shared_ptr<Panel> panel) {
  child_panels_.push_back(panel);
  YGNodeInsertChild(yoga_node_, panel->yoga_node_, YGNodeGetChildCount(yoga_node_));
}

void Panel::RemovePanel(std::shared_ptr<Panel> panel) {
  auto it = std::find(child_panels_.begin(), child_panels_.end(), panel);
  if (it != child_panels_.end()) {
    size_t index = std::distance(child_panels_.begin(), it);
    YGNodeRemoveChild(yoga_node_, panel->yoga_node_);
    child_panels_.erase(it);
  }
}

void Panel::ClearPanels() {
  while (YGNodeGetChildCount(yoga_node_) > 0) {
    YGNodeRemoveChild(yoga_node_, YGNodeGetChild(yoga_node_, 0));
  }
  child_panels_.clear();
}

void Panel::OnResize(int rows, int cols, int y, int x) {
  if (auto_resize_) {
    region_.x = x;
    region_.y = y;
    region_.width = cols;
    region_.height = rows;
    
    YGNodeStyleSetWidth(yoga_node_, cols);
    YGNodeStyleSetHeight(yoga_node_, rows);
  }
  
  CalcContentRegion();
  CalculateLayout();
  ApplyLayout();
}

void Panel::CalculateLayout() {
  // Calculate layout using Yoga
  YGNodeCalculateLayout(
    yoga_node_,
    region_.width,
    region_.height,
    YGDirectionLTR
  );
}

void Panel::ApplyLayout() {
  // Apply calculated layout to this panel
  region_.x = YGNodeLayoutGetLeft(yoga_node_);
  region_.y = YGNodeLayoutGetTop(yoga_node_);
  region_.width = YGNodeLayoutGetWidth(yoga_node_);
  region_.height = YGNodeLayoutGetHeight(yoga_node_);

  // Apply layout to child panels
  for (auto& panel : child_panels_) {
    panel->OnResize(
      YGNodeLayoutGetHeight(panel->yoga_node_),
      YGNodeLayoutGetWidth(panel->yoga_node_),
      YGNodeLayoutGetTop(panel->yoga_node_) + region_.y,
      YGNodeLayoutGetLeft(panel->yoga_node_) + region_.x
    );
  }
}

void Panel::OnDraw() {
  // Draw border if enabled
  if (has_border_) {
    // Create a new window for the panel
    WINDOW* win = newwin(region_.height, region_.width, region_.y, region_.x);
    box(win, 0, 0);

    // Draw title if present
    if (!title_.empty()) {
      mvwprintw(win, 0, (region_.width - title_.size()) / 2, 
                " %s ", title_.c_str());
    }

    wrefresh(win);
    delwin(win);
  }

  // Draw child panels
  for (auto& panel : child_panels_) {
    panel->OnDraw();
  }
}

void Panel::CalcContentRegion() {
  // Calculate the available region for content, accounting for borders
  int content_x = region_.x + (has_border_ ? 1 : 0);
  int content_y = region_.y + (has_border_ ? 1 : 0);
  int content_width = region_.width - (has_border_ ? 2 : 0);
  int content_height = region_.height - (has_border_ ? 2 : 0);

  // Ensure minimum size
  content_width = std::max(content_width, 1);
  content_height = std::max(content_height, 1);

  // Store the content region for layout calculations
  region_.x = content_x;
  region_.y = content_y;
  region_.width = content_width;
  region_.height = content_height;
}

} // namespace swviz
} // namespace xmotion 