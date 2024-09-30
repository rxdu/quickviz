/*
 * @file yoga_utils.cpp
 * @date 9/30/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "yoga_utils.hpp"

namespace quickviz {
YGAlign YogaUtils::ToYogaAlign(Styling::AlignContent align) {
  switch (align) {
    case Styling::AlignContent::kFlexStart:
      return YGAlignFlexStart;
    case Styling::AlignContent::kFlexEnd:
      return YGAlignFlexEnd;
    case Styling::AlignContent::kStretch:
      return YGAlignStretch;
    case Styling::AlignContent::kCenter:
      return YGAlignCenter;
    case Styling::AlignContent::kSpaceBetween:
      return YGAlignSpaceBetween;
    case Styling::AlignContent::kSpaceAround:
      return YGAlignSpaceAround;
    case Styling::AlignContent::kSpaceEvenly:
      return YGAlignSpaceEvenly;
  }
  return YGAlignFlexStart;
}

YGAlign YogaUtils::ToYogaAlign(Styling::AlignItems align) {
  switch (align) {
    case Styling::AlignItems::kStretch:
      return YGAlignStretch;
    case Styling::AlignItems::kFlexStart:
      return YGAlignFlexStart;
    case Styling::AlignItems::kFlexEnd:
      return YGAlignFlexEnd;
    case Styling::AlignItems::kCenter:
      return YGAlignCenter;
    case Styling::AlignItems::kBaseline:
      return YGAlignBaseline;
  }
  return YGAlignStretch;
}

YGDisplay YogaUtils::ToYogaDisplay(Styling::Display display) {
  switch (display) {
    case Styling::Display::kFlex:
      return YGDisplayFlex;
    case Styling::Display::kNone:
      return YGDisplayNone;
  }
  return YGDisplayFlex;
}

YGFlexDirection YogaUtils::ToYogaFlexDirection(
    Styling::FlexDirection direction) {
  switch (direction) {
    case Styling::FlexDirection::kColumn:
      return YGFlexDirectionColumn;
    case Styling::FlexDirection::kRow:
      return YGFlexDirectionRow;
    case Styling::FlexDirection::kColumnReverse:
      return YGFlexDirectionColumnReverse;
    case Styling::FlexDirection::kRowReverse:
      return YGFlexDirectionRowReverse;
  }
  return YGFlexDirectionColumn;
}

YGWrap YogaUtils::ToYogaFlexWrap(Styling::FlexWrap wrap) {
  switch (wrap) {
    case Styling::FlexWrap::kNoWrap:
      return YGWrapNoWrap;
    case Styling::FlexWrap::kWrap:
      return YGWrapWrap;
    case Styling::FlexWrap::kWrapReverse:
      return YGWrapWrapReverse;
  }
  return YGWrapNoWrap;
}

YGJustify YogaUtils::ToYogaJustify(Styling::JustifyContent justify) {
  switch (justify) {
    case Styling::JustifyContent::kFlexStart:
      return YGJustifyFlexStart;
    case Styling::JustifyContent::kFlexEnd:
      return YGJustifyFlexEnd;
    case Styling::JustifyContent::kCenter:
      return YGJustifyCenter;
    case Styling::JustifyContent::kSpaceBetween:
      return YGJustifySpaceBetween;
    case Styling::JustifyContent::kSpaceAround:
      return YGJustifySpaceAround;
    case Styling::JustifyContent::kSpaceEvenly:
      return YGJustifySpaceEvenly;
  }
  return YGJustifyFlexStart;
}

YGEdge YogaUtils::ToYogaEdge(Styling::Edge edge) {
  switch (edge) {
    case Styling::Edge::kEdgeLeft:
      return YGEdgeLeft;
    case Styling::Edge::kEdgeTop:
      return YGEdgeTop;
    case Styling::Edge::kEdgeRight:
      return YGEdgeRight;
    case Styling::Edge::kEdgeBottom:
      return YGEdgeBottom;
    case Styling::Edge::kEdgeStart:
      return YGEdgeStart;
    case Styling::Edge::kEdgeEnd:
      return YGEdgeEnd;
    case Styling::Edge::kEdgeHorizontal:
      return YGEdgeHorizontal;
    case Styling::Edge::kEdgeVertical:
      return YGEdgeVertical;
    case Styling::Edge::kEdgeAll:
      return YGEdgeAll;
  }
  return YGEdgeLeft;
}

YGDirection YogaUtils::ToYogaDirection(Styling::LayoutDirection direction) {
  switch (direction) {
    case Styling::LayoutDirection::kLTR:
      return YGDirectionLTR;
    case Styling::LayoutDirection::kRTL:
      return YGDirectionRTL;
  }
  return YGDirectionLTR;
}

YGPositionType YogaUtils::ToYogaPositionType(Styling::PositionType type) {
  switch (type) {
    case Styling::PositionType::kRelative:
      return YGPositionTypeRelative;
    case Styling::PositionType::kAbsolute:
      return YGPositionTypeAbsolute;
    case Styling::PositionType::kStatic:
      return YGPositionTypeStatic;
  }
  return YGPositionTypeRelative;
}
}  // namespace quickviz