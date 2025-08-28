/*
 * scene_view_panel.cpp
 *
 * Created on August 27, 2025
 * Description: ImGui integration panel for GlSceneManager
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/gl_scene_panel.hpp"

#include <imgui.h>
#include "imview/fonts.hpp"
#include "imview/input/mouse.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/details/selection_manager.hpp"

namespace quickviz {

GlScenePanel::GlScenePanel(const std::string& name,
                               GlSceneManager::Mode mode)
    : Panel(name) {
  scene_manager_ = std::make_unique<GlSceneManager>(name + "_manager", mode);
}

void GlScenePanel::Draw() {
  Begin();
  RenderInsideWindow();
  End();
}

void GlScenePanel::RenderInsideWindow() {
  // Handle input processing
  HandleInput();

  // Get current content region
  ImVec2 content_size = ImGui::GetContentRegionAvail();

  // Render the scene to framebuffer
  scene_manager_->RenderToFramebuffer(content_size.x, content_size.y);

  // Display the framebuffer texture in ImGui
  uint32_t texture_id = scene_manager_->GetFramebufferTexture();
  if (texture_id != 0) {
    ImVec2 uv0(0, 1);  // Bottom-left (Y flipped for OpenGL)
    ImVec2 uv1(1, 0);  // Top-right (Y flipped for OpenGL)
    ImGui::Image((void*)(intptr_t)texture_id, content_size, uv0, uv1);
  }

  // Render info overlay if enabled
  if (show_rendering_info_) {
    RenderInfoOverlay();
  }
}

void GlScenePanel::SetShowRenderingInfo(bool show) {
  show_rendering_info_ = show;
}

void GlScenePanel::SetBackgroundColor(float r, float g, float b, float a) {
  scene_manager_->SetBackgroundColor(r, g, b, a);
}

// Delegate GlSceneManager methods
GlSceneManager::Mode GlScenePanel::GetMode() const {
  return scene_manager_->GetMode();
}

void GlScenePanel::SetClippingPlanes(float z_near, float z_far) {
  scene_manager_->SetClippingPlanes(z_near, z_far);
}

void GlScenePanel::AddOpenGLObject(const std::string& name, std::unique_ptr<OpenGlObject> object) {
  scene_manager_->AddOpenGLObject(name, std::move(object));
}

void GlScenePanel::RemoveOpenGLObject(const std::string& name) {
  scene_manager_->RemoveOpenGLObject(name);
}

OpenGlObject* GlScenePanel::GetOpenGLObject(const std::string& name) {
  return scene_manager_->GetOpenGLObject(name);
}

void GlScenePanel::ClearOpenGLObjects() {
  scene_manager_->ClearOpenGLObjects();
}

void GlScenePanel::SetPreDrawCallback(GlSceneManager::PreDrawCallback callback) {
  scene_manager_->SetPreDrawCallback(std::move(callback));
}

void GlScenePanel::EnableCoordinateSystemTransformation(bool enable) {
  scene_manager_->EnableCoordinateSystemTransformation(enable);
}

bool GlScenePanel::IsCoordinateSystemTransformationEnabled() const {
  return scene_manager_->IsCoordinateSystemTransformationEnabled();
}

// Camera access delegation
CameraController* GlScenePanel::GetCameraController() const {
  return scene_manager_->GetCameraController();
}

Camera* GlScenePanel::GetCamera() const {
  return scene_manager_->GetCamera();
}

const glm::mat4& GlScenePanel::GetProjectionMatrix() const {
  return scene_manager_->GetProjectionMatrix();
}

const glm::mat4& GlScenePanel::GetViewMatrix() const {
  return scene_manager_->GetViewMatrix();
}

const glm::mat4& GlScenePanel::GetCoordinateTransform() const {
  return scene_manager_->GetCoordinateTransform();
}

// === Selection System Implementation ===

SelectionManager& GlScenePanel::GetSelection() {
  return scene_manager_->GetSelection();
}

const SelectionManager& GlScenePanel::GetSelection() const {
  return scene_manager_->GetSelection();
}

SelectionResult GlScenePanel::Select(float screen_x, float screen_y, const SelectionOptions& options) {
  return scene_manager_->Select(screen_x, screen_y, options);
}

bool GlScenePanel::AddToSelection(float screen_x, float screen_y, const SelectionOptions& options) {
  return scene_manager_->AddToSelection(screen_x, screen_y, options);
}

const MultiSelection& GlScenePanel::GetMultiSelection() const {
  return scene_manager_->GetMultiSelection();
}

void GlScenePanel::ClearSelection() {
  scene_manager_->GetSelection().ClearSelection();
}

void GlScenePanel::HandleInput() {
  // Only process input when window is hovered and has focus
  if (!ImGui::IsWindowHovered()) {
    // Reset mouse button state when mouse is outside the window
    scene_manager_->GetCameraController()->SetActiveMouseButton(
        MouseButton::kNone);
    return;
  }

  ImGuiIO& io = ImGui::GetIO();

  // Handle mouse clicks for selection (pass coordinates to scene manager)
  if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    ImVec2 mouse_pos = ImGui::GetMousePos();
    ImVec2 window_pos = ImGui::GetWindowPos();
    ImVec2 window_content_min = ImGui::GetWindowContentRegionMin();

    // Convert to relative coordinates within the content area
    float relative_x = mouse_pos.x - window_pos.x - window_content_min.x;
    float relative_y = mouse_pos.y - window_pos.y - window_content_min.y;

    // Track selection calls (removed debug output)

    // Use new SelectionManager API for unified selection
    scene_manager_->Select(relative_x, relative_y);
  }

  // Only process mouse delta when ImGui wants to capture mouse
  if (ImGui::IsMousePosValid() && io.WantCaptureMouse) {
    // Check for mouse buttons and update camera controller state
    int active_button = MouseButton::kNone;

    if (ImGui::IsMouseDown(MouseButton::kLeft)) {
      active_button = MouseButton::kLeft;
    } else if (ImGui::IsMouseDown(MouseButton::kMiddle)) {
      active_button = MouseButton::kMiddle;
    } else if (ImGui::IsMouseDown(MouseButton::kRight)) {
      active_button = MouseButton::kRight;
    }

    // Set the active mouse button in the camera controller
    scene_manager_->GetCameraController()->SetActiveMouseButton(active_button);

    // Process mouse movement if any button is pressed
    if (active_button != MouseButton::kNone) {
      scene_manager_->GetCameraController()->ProcessMouseMovement(
          io.MouseDelta.x, io.MouseDelta.y);
    }

    // Track mouse wheel scroll
    scene_manager_->GetCameraController()->ProcessMouseScroll(io.MouseWheel);
  } else {
    // Reset mouse button state when not capturing mouse
    scene_manager_->GetCameraController()->SetActiveMouseButton(
        MouseButton::kNone);
  }
}

void GlScenePanel::RenderInfoOverlay() {
  ImVec2 content_size = ImGui::GetContentRegionAvail();

  // Draw FPS info at the bottom of the scene
  ImGui::SetCursorPos(ImVec2(10, content_size.y - 25));
  ImGui::PushFont(Fonts::GetFont(FontSize::kFont18));
  ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(0, 153, 153, 200));
  ImGui::Text("FPS: %.1f, %.3f ms/frame", ImGui::GetIO().Framerate,
              1000.0f / ImGui::GetIO().Framerate);
  ImGui::PopStyleColor();
  ImGui::PopFont();
}

}  // namespace quickviz