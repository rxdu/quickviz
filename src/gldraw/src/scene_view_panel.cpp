/*
 * scene_view_panel.cpp
 *
 * Created on August 27, 2025
 * Description: ImGui integration panel for GlSceneManager
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/scene_view_panel.hpp"

#include <imgui.h>
#include "imview/fonts.hpp"
#include "imview/input/mouse.hpp"
#include "gldraw/renderable/point_cloud.hpp"

namespace quickviz {

SceneViewPanel::SceneViewPanel(const std::string& name,
                               GlSceneManager::Mode mode)
    : Panel(name) {
  scene_manager_ = std::make_unique<GlSceneManager>(name + "_manager", mode);
}

void SceneViewPanel::Draw() {
  Begin();
  RenderInsideWindow();
  End();
}

void SceneViewPanel::RenderInsideWindow() {
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

void SceneViewPanel::SetShowRenderingInfo(bool show) {
  show_rendering_info_ = show;
}

void SceneViewPanel::SetBackgroundColor(float r, float g, float b, float a) {
  scene_manager_->SetBackgroundColor(r, g, b, a);
}

// Delegate GlSceneManager methods
GlSceneManager::Mode SceneViewPanel::GetMode() const {
  return scene_manager_->GetMode();
}

void SceneViewPanel::SetClippingPlanes(float z_near, float z_far) {
  scene_manager_->SetClippingPlanes(z_near, z_far);
}

void SceneViewPanel::AddOpenGLObject(const std::string& name, std::unique_ptr<OpenGlObject> object) {
  scene_manager_->AddOpenGLObject(name, std::move(object));
}

void SceneViewPanel::RemoveOpenGLObject(const std::string& name) {
  scene_manager_->RemoveOpenGLObject(name);
}

OpenGlObject* SceneViewPanel::GetOpenGLObject(const std::string& name) {
  return scene_manager_->GetOpenGLObject(name);
}

void SceneViewPanel::ClearOpenGLObjects() {
  scene_manager_->ClearOpenGLObjects();
}

void SceneViewPanel::SetPreDrawCallback(GlSceneManager::PreDrawCallback callback) {
  scene_manager_->SetPreDrawCallback(std::move(callback));
}

void SceneViewPanel::EnableCoordinateSystemTransformation(bool enable) {
  scene_manager_->EnableCoordinateSystemTransformation(enable);
}

bool SceneViewPanel::IsCoordinateSystemTransformationEnabled() const {
  return scene_manager_->IsCoordinateSystemTransformationEnabled();
}

// Camera access delegation
CameraController* SceneViewPanel::GetCameraController() const {
  return scene_manager_->GetCameraController();
}

Camera* SceneViewPanel::GetCamera() const {
  return scene_manager_->GetCamera();
}

const glm::mat4& SceneViewPanel::GetProjectionMatrix() const {
  return scene_manager_->GetProjectionMatrix();
}

const glm::mat4& SceneViewPanel::GetViewMatrix() const {
  return scene_manager_->GetViewMatrix();
}

const glm::mat4& SceneViewPanel::GetCoordinateTransform() const {
  return scene_manager_->GetCoordinateTransform();
}

GlSceneManager::MouseRay SceneViewPanel::GetMouseRayInWorldSpace(float mouse_x, float mouse_y, 
                                                                 float window_width, float window_height) const {
  return scene_manager_->GetMouseRayInWorldSpace(mouse_x, mouse_y, window_width, window_height);
}

// GPU picking delegation
size_t SceneViewPanel::PickPointAtPixel(int x, int y, const std::string& point_cloud_name) {
  return scene_manager_->PickPointAtPixel(x, y, point_cloud_name);
}

size_t SceneViewPanel::PickPointAtPixelWithRadius(int x, int y, int radius, const std::string& point_cloud_name) {
  return scene_manager_->PickPointAtPixelWithRadius(x, y, radius, point_cloud_name);
}

// Point cloud selection delegation
void SceneViewPanel::SetActivePointCloud(PointCloud* point_cloud) {
  scene_manager_->SetActivePointCloud(point_cloud);
}

PointCloud* SceneViewPanel::GetActivePointCloud() const {
  return scene_manager_->GetActivePointCloud();
}

// Point selection operations delegation
bool SceneViewPanel::SelectPointAt(float screen_x, float screen_y, int radius) {
  return scene_manager_->SelectPointAt(screen_x, screen_y, radius);
}

bool SceneViewPanel::AddPointAt(float screen_x, float screen_y, int radius) {
  return scene_manager_->AddPointAt(screen_x, screen_y, radius);
}

bool SceneViewPanel::TogglePointAt(float screen_x, float screen_y, int radius) {
  return scene_manager_->TogglePointAt(screen_x, screen_y, radius);
}

void SceneViewPanel::ClearPointSelection() {
  scene_manager_->ClearPointSelection();
}

const std::vector<size_t>& SceneViewPanel::GetSelectedPointIndices() const {
  return scene_manager_->GetSelectedPointIndices();
}

size_t SceneViewPanel::GetSelectedPointCount() const {
  return scene_manager_->GetSelectedPointCount();
}

glm::vec3 SceneViewPanel::GetSelectionCentroid() const {
  return scene_manager_->GetSelectionCentroid();
}

std::pair<glm::vec3, glm::vec3> SceneViewPanel::GetSelectionBounds() const {
  return scene_manager_->GetSelectionBounds();
}

// Selection visualization delegation
void SceneViewPanel::SetSelectionVisualization(const glm::vec3& color,
                                               float size_multiplier,
                                               const std::string& layer_name) {
  scene_manager_->SetSelectionVisualization(color, size_multiplier, layer_name);
}

void SceneViewPanel::SetSelectionVisualizationEnabled(bool enabled) {
  scene_manager_->SetSelectionVisualizationEnabled(enabled);
}

void SceneViewPanel::SetPointSelectionCallback(GlSceneManager::PointSelectionCallback callback) {
  scene_manager_->SetPointSelectionCallback(std::move(callback));
}

// Object selection delegation
void SceneViewPanel::SelectObjectAt(float screen_x, float screen_y) {
  scene_manager_->SelectObjectAt(screen_x, screen_y);
}

const std::string& SceneViewPanel::GetSelectedObjectName() const {
  return scene_manager_->GetSelectedObjectName();
}

void SceneViewPanel::ClearObjectSelection() {
  scene_manager_->ClearObjectSelection();
}

void SceneViewPanel::SetObjectHighlight(const std::string& name, bool highlighted) {
  scene_manager_->SetObjectHighlight(name, highlighted);
}

void SceneViewPanel::SetObjectSelectionCallback(GlSceneManager::ObjectSelectionCallback callback) {
  scene_manager_->SetObjectSelectionCallback(std::move(callback));
}

void SceneViewPanel::HandleInput() {
  // Only process input when window is hovered and has focus
  if (!ImGui::IsWindowHovered()) {
    // Reset mouse button state when mouse is outside the window
    scene_manager_->GetCameraController()->SetActiveMouseButton(
        MouseButton::kNone);
    return;
  }

  ImGuiIO& io = ImGui::GetIO();

  // Handle mouse clicks for object selection (regardless of WantCaptureMouse)
  if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    ImVec2 mouse_pos = ImGui::GetMousePos();
    ImVec2 window_pos = ImGui::GetWindowPos();
    ImVec2 window_content_min = ImGui::GetWindowContentRegionMin();
    
    // Convert to relative coordinates within the content area
    float relative_x = mouse_pos.x - window_pos.x - window_content_min.x;
    float relative_y = mouse_pos.y - window_pos.y - window_content_min.y;
    
    // Delegate to scene manager for object selection
    scene_manager_->SelectObjectAt(relative_x, relative_y);
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

void SceneViewPanel::RenderInfoOverlay() {
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