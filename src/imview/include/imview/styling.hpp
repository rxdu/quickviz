/*
 * @file styling.hpp
 * @date 9/30/24
 * @brief Styling properties for flexbox layout with yoga
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */
#ifndef QUICKVIZ_STYLING_HPP
#define QUICKVIZ_STYLING_HPP

namespace quickviz {
namespace Styling {
enum class AlignContent {
  kFlexStart = 0,
  kFlexEnd,
  kStretch,
  kCenter,
  kSpaceBetween,
  kSpaceAround,
  kSpaceEvenly
};

enum class AlignItems {
  kStretch = 0,
  kFlexStart,
  kFlexEnd,
  kCenter,
  kBaseline
};

enum class Display { kFlex = 0, kNone };
enum class FlexDirection { kColumn = 0, kRow, kColumnReverse, kRowReverse };
enum class FlexWrap { kNoWrap = 0, kWrap, kWrapReverse };

enum class Edge {
  kEdgeLeft,
  kEdgeTop,
  kEdgeRight,
  kEdgeBottom,
  kEdgeStart,
  kEdgeEnd,
  kEdgeHorizontal,
  kEdgeVertical,
  kEdgeAll
};

enum class JustifyContent {
  kFlexStart = 0,
  kFlexEnd,
  kCenter,
  kSpaceBetween,
  kSpaceAround,
  kSpaceEvenly
};

enum class LayoutDirection { kLTR = 0, kRTL };
enum class PositionType { kRelative = 0, kAbsolute, kStatic };
}  // namespace Styling
}  // namespace quickviz

#endif  // QUICKVIZ_STYLING_HPP
