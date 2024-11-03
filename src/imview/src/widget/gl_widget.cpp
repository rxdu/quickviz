/*
 * @file gl_widget.cpp
 * @date 10/29/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "imview/widget/gl_widget.hpp"

namespace quickviz {
GlWidget::GlWidget(const std::string& widget_name) : Panel(widget_name) {
  this->SetAutoLayout(false);
  this->SetWindowNoMenuButton();
  this->SetNoBackground(true);

  frame_buffer_ = std::make_unique<FrameBuffer>(100, 50);
}

void GlWidget::SetGlRenderFunction(GlWidget::GlRenderFunction func) {
  render_function_ = func;
}

void GlWidget::Draw() {
  Begin();
  {
    ImVec2 content_size = ImGui::GetContentRegionAvail();
    float width = content_size.x;
    float height = content_size.y;

    frame_buffer_->Resize(width, height);
    frame_buffer_->Bind();
    if (render_function_ != nullptr) render_function_(*frame_buffer_.get());
    frame_buffer_->Unbind();

    ImVec2 uv0 = ImVec2(0, 1);
    ImVec2 uv1 = ImVec2(1, 0);
    ImVec4 tint_col = ImVec4(1, 1, 1, 1);
    ImVec4 border_col = ImVec4(0, 0, 0, 0);
    ImGui::Image((void*)(intptr_t)frame_buffer_->GetTextureId(),
                 ImVec2(width, height), uv0, uv1, tint_col, border_col);
  }
  End();
}
}  // namespace quickviz