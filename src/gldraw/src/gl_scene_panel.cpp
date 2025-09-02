/*
 * scene_view_panel.cpp
 *
 * Created on August 27, 2025
 * Description: ImGui integration panel for GlSceneManager
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/gl_scene_panel.hpp"

#include <cstdio>
#include <iostream>

#include "imgui.h"
#include "imview/fonts.hpp"

#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/selection_manager.hpp"

namespace quickviz {

GlScenePanel::GlScenePanel(const std::string& name, SceneManager::Mode mode)
    : Panel(name) {
  scene_manager_ = std::make_unique<SceneManager>(name + "_manager", mode);

  // Create and register the 3D scene input handler
  scene_input_handler_ =
      SceneInputHandlerFactory::CreateStandard(scene_manager_.get());

  // Register with panel's input manager when it becomes available
  // This will be done in SetInputManager or when the panel is added to a viewer
}

void GlScenePanel::Draw() {
  Begin();
  RenderInsideWindow();
  End();
}

void GlScenePanel::RenderInsideWindow() {
  // Get current content region BEFORE rendering the image
  ImVec2 content_size = ImGui::GetContentRegionAvail();

  // Save image position for overlay rendering
  ImVec2 image_pos = ImGui::GetCursorScreenPos();

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
  // Pass the saved content_size and image_pos to avoid recalculating with
  // invalid values
  if (show_rendering_info_) {
    RenderInfoOverlay(content_size, image_pos);
  }
}

void GlScenePanel::SetShowRenderingInfo(bool show) {
  show_rendering_info_ = show;
}

void GlScenePanel::SetBackgroundColor(float r, float g, float b, float a) {
  scene_manager_->SetBackgroundColor(r, g, b, a);
}

// Delegate GlSceneManager methods
SceneManager::Mode GlScenePanel::GetMode() const {
  return scene_manager_->GetMode();
}

void GlScenePanel::SetClippingPlanes(float z_near, float z_far) {
  scene_manager_->SetClippingPlanes(z_near, z_far);
}

void GlScenePanel::AddOpenGLObject(const std::string& name,
                                   std::unique_ptr<OpenGlObject> object) {
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

void GlScenePanel::SetPreDrawCallback(
    SceneManager::PreDrawCallback callback) {
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

Camera* GlScenePanel::GetCamera() const { return scene_manager_->GetCamera(); }

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

SelectionResult GlScenePanel::Select(float screen_x, float screen_y,
                                     const SelectionOptions& options) {
  return scene_manager_->Select(screen_x, screen_y, options);
}

bool GlScenePanel::AddToSelection(float screen_x, float screen_y,
                                  const SelectionOptions& options) {
  return scene_manager_->AddToSelection(screen_x, screen_y, options);
}

const MultiSelection& GlScenePanel::GetMultiSelection() const {
  return scene_manager_->GetMultiSelection();
}

void GlScenePanel::ClearSelection() {
  scene_manager_->GetSelection().ClearSelection();
}

void GlScenePanel::SetSelectionEnabled(bool enabled) {
  scene_manager_->SetSelectionEnabled(enabled);
}

bool GlScenePanel::IsSelectionEnabled() const {
  return scene_manager_->IsSelectionEnabled();
}

void GlScenePanel::RenderInfoOverlay(const ImVec2& content_size,
                                     const ImVec2& image_pos) {
  // Get window draw list for overlay rendering
  ImDrawList* draw_list = ImGui::GetWindowDrawList();

  // Calculate text position at bottom-left of the rendered scene
  ImVec2 text_pos;
  text_pos.x = image_pos.x + 10;
  text_pos.y = image_pos.y + content_size.y - 25;

  // Format FPS text
  char fps_text[64];
  snprintf(fps_text, sizeof(fps_text), "FPS: %.1f, %.3f ms/frame",
           ImGui::GetIO().Framerate, 1000.0f / ImGui::GetIO().Framerate);

  // Draw text with shadow for better visibility
  ImU32 text_color = IM_COL32(0, 255, 255, 200);  // Cyan color
  ImU32 shadow_color = IM_COL32(0, 0, 0, 150);    // Dark shadow

  // Draw shadow (offset by 1 pixel)
  draw_list->AddText(ImVec2(text_pos.x + 1, text_pos.y + 1), shadow_color,
                     fps_text);

  // Draw main text
  draw_list->AddText(text_pos, text_color, fps_text);
}

// New imview-based input handling methods
bool GlScenePanel::OnInputEvent(const InputEvent& event) {
  // Update viewport size for coordinate transformations
  if (scene_input_handler_) {
    ImVec2 content_size = ImGui::GetContentRegionAvail();
    scene_input_handler_->SetViewportSize(static_cast<int>(content_size.x),
                                          static_cast<int>(content_size.y));

    // Forward event to scene input handler directly
    return scene_input_handler_->OnInputEvent(event);
  }

  // No scene input handler - allow event to propagate
  return false;
}
}  // namespace quickviz