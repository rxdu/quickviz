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
}

void GlWidget::AddOpenGLObject(const std::string& name,
                               std::unique_ptr<OpenGLDrawable> object) {
  drawable_objects_[name] = std::move(object);
}

void GlWidget::RemoveOpenGLObject(const std::string& name) {
  if (drawable_objects_.find(name) != drawable_objects_.end()) {
    drawable_objects_.erase(name);
  }
}

void GlWidget::ClearOpenGLObjects() { drawable_objects_.clear(); }

void GlWidget::UpdateView(const glm::mat4& projection, const glm::mat4& view) {
  projection_ = projection;
  view_ = view;
}

void GlWidget::DrawOpenGLObject() {
  ImVec2 content_size = ImGui::GetContentRegionAvail();
  float width = content_size.x;
  float height = content_size.y;

  if (frame_buffer_ != nullptr) {
    // render to frame buffer
    frame_buffer_->Resize(width, height);
    frame_buffer_->Bind();
    frame_buffer_->Clear();
    for (auto& obj : drawable_objects_) {
      obj.second->OnDraw(projection_, view_);
    }
    frame_buffer_->Unbind();

    // render frame buffer to ImGui
    ImVec2 uv0 = ImVec2(0, 1);
    ImVec2 uv1 = ImVec2(1, 0);
    ImVec4 tint_col = ImVec4(1, 1, 1, 1);
    ImVec4 border_col = ImVec4(0, 0, 0, 0);
    ImGui::Image((void*)(intptr_t)frame_buffer_->GetTextureId(),
                 ImVec2(width, height), uv0, uv1, tint_col, border_col);
  } else {
    frame_buffer_ = std::make_unique<FrameBuffer>(width, height);
  }
}

void GlWidget::Draw() {
  Begin();
  DrawOpenGLObject();
  End();
}
}  // namespace quickviz