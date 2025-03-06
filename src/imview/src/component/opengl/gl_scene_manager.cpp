/*
 * gl_scene_manager.cpp
 *
 * Created on 3/6/25 9:09 PM
 * Description:
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "imview/component/opengl/gl_scene_manager.hpp"

#include <stdexcept>

#include "imview/fonts.hpp"

namespace quickviz {
GlSceneManager::GlSceneManager(const std::string& name) : Panel(name) {
  this->SetAutoLayout(false);
  this->SetWindowNoMenuButton();
  //   this->SetNoBackground(true);

  camera_ = std::make_unique<Camera>();
  camera_controller_ = std::make_unique<CameraController>(
      *camera_, glm::vec3(0.0f, 6.0f, 8.0f), 0.0f, 25.0f);
}

void GlSceneManager::SetShowRenderingInfo(bool show) {
  show_rendering_info_ = show;
}

void GlSceneManager::AddOpenGLObject(const std::string& name,
                                     std::unique_ptr<OpenGLDrawable> object) {
  if (object == nullptr) {
    throw std::invalid_argument("Object is nullptr");
  }
  drawable_objects_[name] = std::move(object);
}

void GlSceneManager::RemoveOpenGLObject(const std::string& name) {
  if (drawable_objects_.find(name) != drawable_objects_.end()) {
    drawable_objects_.erase(name);
  }
}

void GlSceneManager::ClearOpenGLObjects() { drawable_objects_.clear(); }

void GlSceneManager::UpdateView(const glm::mat4& projection,
                                const glm::mat4& view) {
  projection_ = projection;
  view_ = view;
}

void GlSceneManager::DrawOpenGLObject() {
  ImVec2 content_size = ImGui::GetContentRegionAvail();
  float width = content_size.x;
  float height = content_size.y;

  if (frame_buffer_ != nullptr) {
    if (frame_buffer_->GetWidth() != width ||
        frame_buffer_->GetHeight() != height) {
      frame_buffer_->Resize(width, height);
    }
    // render to frame buffer
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

void GlSceneManager::Draw() {
  Begin();

  // update view according to user input
  ImGuiIO& io = ImGui::GetIO();
  ImVec2 content_size = ImGui::GetContentRegionAvail();

  // only process mouse delta when mouse position is within the scene panel
  if (ImGui::IsMousePosValid() && io.WantCaptureMouse &&
      ImGui::IsWindowHovered()) {
    // track mouse move delta only when the mouse left button is pressed
    if (ImGui::IsMouseDown(MouseButton::kLeft)) {
      camera_controller_->ProcessMouseMovement(io.MouseDelta.x,
                                               io.MouseDelta.y);
    }

    // track mouse wheel scroll
    camera_controller_->ProcessMouseScroll(io.MouseWheel);
  }

  // get view matrices from camera
  float aspect_ratio =
      static_cast<float>(content_size.x) / static_cast<float>(content_size.y);
  glm::mat4 projection = camera_->GetProjectionMatrix(aspect_ratio);
  glm::mat4 view = camera_->GetViewMatrix();
  UpdateView(projection, view);

  // finally draw the scene
  DrawOpenGLObject();

  // draw frame rate at the bottom of the scene
  if (show_rendering_info_) {
    ImGui::SetCursorPos(ImVec2(10, content_size.y - 25));
    ImGui::PushFont(Fonts::GetFont(FontSize::kFont18));
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 153, 153, 200));
    ImGui::Text("FPS: %.1f, %.3f ms/frame", ImGui::GetIO().Framerate,
                1000.0f / ImGui::GetIO().Framerate);
    ImGui::PopStyleColor();
    ImGui::PopFont();
  }

  End();
}
}  // namespace quickviz