/*
 * @file ui_panel.cpp
 * @date 10/1/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/ui_panel.hpp"

#include "imgui.h"

#ifdef IMVIEW_WITH_GLAD
#include <glad/glad.h>
#endif
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

namespace quickviz {
UiPanel::UiPanel(std::string name) : Panel(name) {}

void UiPanel::SetPosition(float x, float y) { Panel::SetPosition(x, y); }

void UiPanel::OnResize(float width, float height) {
  Panel::OnResize(width, height);
}

void UiPanel::OnRender() {
  if (!visible_) return;
  for (auto renderable : renderables_) {
    if (renderable->IsVisible()) renderable->OnRender();
  }
}
}  // namespace quickviz