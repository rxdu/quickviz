/*
 * @file panel.cpp
 * @date 9/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/panel.hpp"

namespace quickviz {
Panel::Panel(std::string name) : name_(name) {}

Panel::~Panel() {}

void Panel::OnResize(int width, int height) {}
}  // namespace quickviz