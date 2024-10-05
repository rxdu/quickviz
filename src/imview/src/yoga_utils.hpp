/*
 * @file yoga_utils.hpp
 * @date 9/30/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_YOGA_UTILS_HPP
#define QUICKVIZ_YOGA_UTILS_HPP

#include "yoga/Yoga.h"
#include "imview/styling.hpp"

namespace quickviz {
namespace YogaUtils {
YGAlign ToYogaAlign(Styling::AlignContent align);
YGAlign ToYogaAlign(Styling::AlignItems align);
YGDisplay ToYogaDisplay(Styling::Display display);
YGFlexDirection ToYogaFlexDirection(Styling::FlexDirection direction);
YGWrap ToYogaFlexWrap(Styling::FlexWrap wrap);
YGJustify ToYogaJustify(Styling::JustifyContent justify);
YGEdge ToYogaEdge(Styling::Edge edge);
YGDirection ToYogaDirection(Styling::LayoutDirection direction);
YGPositionType ToYogaPositionType(Styling::PositionType type);
};  // namespace YogaUtils
}  // namespace quickviz

#endif  // QUICKVIZ_YOGA_UTILS_HPP