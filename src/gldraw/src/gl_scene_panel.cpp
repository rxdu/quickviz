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
#include "imview/input/mouse.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/selection_manager.hpp"

namespace quickviz {

GlScenePanel::GlScenePanel(const std::string& name, GlSceneManager::Mode mode)
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
  if (use_enhanced_input_) {
    HandleInputEnhanced();
  } else {
    HandleInput();
  }

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
GlSceneManager::Mode GlScenePanel::GetMode() const {
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
    GlSceneManager::PreDrawCallback callback) {
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

void GlScenePanel::HandleInputEnhanced() {
  // Only process input when window is hovered
  if (!ImGui::IsWindowHovered()) {
    scene_manager_->GetCameraController()->SetActiveMouseButton(
        MouseButton::kNone);
    return;
  }

  ImGuiIO& io = ImGui::GetIO();

  // Handle mouse press events
  for (int button = 0; button < 3; ++button) {
    if (ImGui::IsMouseClicked(button)) {
      auto event = CreateInputEvent(InputEventType::kMousePress, button);
      
      // Get mouse position relative to content area
      ImVec2 mouse_pos = ImGui::GetMousePos();
      ImVec2 window_pos = ImGui::GetWindowPos();
      ImVec2 window_content_min = ImGui::GetWindowContentRegionMin();
      float relative_x = mouse_pos.x - window_pos.x - window_content_min.x;
      float relative_y = mouse_pos.y - window_pos.y - window_content_min.y;
      event->SetScreenPosition(glm::vec2(relative_x, relative_y));
      
      // Dispatch through the enhanced system
      if (!input_dispatcher_.DispatchEvent(event)) {
        // If not consumed, check for mapped actions
        auto actions = input_mapping_.GetActionsForEvent(*event);
        for (const auto& action : actions) {
          if (action == Actions::SELECT_SINGLE) {
            scene_manager_->Select(relative_x, relative_y);
          } else if (action == Actions::SELECT_ADD) {
            scene_manager_->AddToSelection(relative_x, relative_y);
          } else if (action == Actions::CAMERA_ROTATE && button == 1) {
            scene_manager_->GetCameraController()->SetActiveMouseButton(
                MouseButton::kRight);
          } else if (action == Actions::CAMERA_PAN && button == 2) {
            scene_manager_->GetCameraController()->SetActiveMouseButton(
                MouseButton::kMiddle);
          }
        }
        
        // Legacy fallback for unmapped left click
        if (actions.empty() && button == 0) {
          scene_manager_->Select(relative_x, relative_y);
        }
      }
    }
    
    if (ImGui::IsMouseReleased(button)) {
      auto event = CreateInputEvent(InputEventType::kMouseRelease, button);
      input_dispatcher_.DispatchEvent(event);
    }
  }

  // Handle mouse movement and dragging
  if (ImGui::IsMousePosValid() && io.WantCaptureMouse) {
    static glm::vec2 last_mouse_pos;
    glm::vec2 current_mouse_pos(io.MousePos.x, io.MousePos.y);
    glm::vec2 delta(io.MouseDelta.x, io.MouseDelta.y);

    // Check if any button is down for drag events
    bool is_dragging = false;
    int active_button = -1;
    for (int button = 0; button < 3; ++button) {
      if (ImGui::IsMouseDown(button)) {
        is_dragging = true;
        active_button = button;
        break;
      }
    }

    if (is_dragging) {
      auto event = CreateInputEvent(InputEventType::kMouseDrag, active_button);
      event->SetScreenPosition(current_mouse_pos);
      event->SetDelta(delta);
      
      if (!input_dispatcher_.DispatchEvent(event)) {
        // Legacy camera control
        scene_manager_->GetCameraController()->SetActiveMouseButton(active_button);
        scene_manager_->GetCameraController()->ProcessMouseMovement(
            delta.x, delta.y);
      }
    } else {
      // Just mouse move, no buttons down
      auto event = CreateInputEvent(InputEventType::kMouseMove);
      event->SetScreenPosition(current_mouse_pos);
      event->SetDelta(delta);
      input_dispatcher_.DispatchEvent(event);
      
      // Reset camera controller button state
      scene_manager_->GetCameraController()->SetActiveMouseButton(
          MouseButton::kNone);
    }

    last_mouse_pos = current_mouse_pos;
  }

  // Handle mouse wheel
  if (io.MouseWheel != 0) {
    auto event = CreateInputEvent(InputEventType::kMouseWheel);
    event->SetDelta(glm::vec2(0, io.MouseWheel));
    
    if (!input_dispatcher_.DispatchEvent(event)) {
      // Legacy zoom
      scene_manager_->GetCameraController()->ProcessMouseScroll(io.MouseWheel);
    }
  }

  // Handle keyboard events - only check for actual pressed keys
  ImGuiIO& io_ref = ImGui::GetIO();
  for (int key = 0; key < IM_ARRAYSIZE(io_ref.KeysDown); ++key) {
    if (ImGui::IsKeyPressed(static_cast<ImGuiKey>(key))) {
      auto event = CreateInputEvent(InputEventType::kKeyPress, key);
      
      if (!input_dispatcher_.DispatchEvent(event)) {
        // Check for mapped keyboard actions
        auto actions = input_mapping_.GetActionsForEvent(*event);
        for (const auto& action : actions) {
          if (action == Actions::CLEAR_SELECTION) {
            this->ClearSelection();
          }
          // Add more keyboard action handlers as needed
        }
      }
    }
    
    if (ImGui::IsKeyReleased(static_cast<ImGuiKey>(key))) {
      auto event = CreateInputEvent(InputEventType::kKeyRelease, key);
      input_dispatcher_.DispatchEvent(event);
    }
  }
}

std::shared_ptr<InputEvent> GlScenePanel::CreateInputEvent(InputEventType type, int button_or_key) {
  auto event = std::make_shared<InputEvent>(type, button_or_key);
  event->SetModifiers(GetCurrentModifiers());
  return event;
}

ModifierKeys GlScenePanel::GetCurrentModifiers() {
  ImGuiIO& io = ImGui::GetIO();
  ModifierKeys mods;
  mods.ctrl = io.KeyCtrl;
  mods.shift = io.KeyShift;
  mods.alt = io.KeyAlt;
  mods.super = io.KeySuper;
  return mods;
}

}  // namespace quickviz