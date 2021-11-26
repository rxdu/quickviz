/*
 * viewer_base.cpp
 *
 * Created on: Jul 27, 2021 08:59
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include "canvas/viewer_base.hpp"

namespace rdu {
namespace wgui {
ViewerBase::ViewerBase(uint32_t width, uint32_t height, std::string title,
                       uint32_t window_hints) {
  wgui::Init();

  window_ = std::unique_ptr<wgui::Window>(
      new wgui::Window(width, height, title, window_hints));
  window_->ApplyDarkStyle();
}

ViewerBase::~ViewerBase() { wgui::Terminate(); }

void ViewerBase::Show() {
  while (!window_->WindowShouldClose()) {
    // handle events
    window_->PollEvents();
    if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Escape))) {
      window_->CloseWindow();
    }

    // draw stuff
    window_->StartNewFrame();

    Draw();

    window_->RenderFrame();
  }
}
}  // namespace wgui
}  // namespace rdu