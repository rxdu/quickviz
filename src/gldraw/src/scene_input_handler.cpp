/*
 * @file scene_input_handler.cpp
 * @date 9/1/25
 * @brief Implementation of 3D scene input handler bridge
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/scene_input_handler.hpp"
#include "gldraw/scene_manager.hpp"
#include "gldraw/camera_control_config.hpp"
#include "gldraw/tools/interaction_tool.hpp"
#include "viewer/input/input_types.hpp"
#include "core/event/input_mapping.hpp"

namespace quickviz {

SceneInputHandler::SceneInputHandler(SceneManager* scene_manager, int priority)
    : scene_manager_(scene_manager), priority_(priority) {
}

bool SceneInputHandler::OnInputEvent(const InputEvent& event) {
  if (!enabled_ || !scene_manager_) {
    return false;
  }

  if (event.IsMouseEvent()) {
    return HandleMouseEvent(event);
  } else if (event.IsKeyboardEvent()) {
    return HandleKeyboardEvent(event);
  }

  return false;
}

bool SceneInputHandler::HandleMouseEvent(const InputEvent& event) {
  bool handled = false;

  // Handle active tool input first (highest priority)
  if (auto active_tool = scene_manager_->GetActiveTool()) {
    if (active_tool->OnInputEvent(event)) {
      return true;  // Tool consumed the event
    }
  }

  // Handle camera control second (lower priority in terms of consumption)
  if (camera_control_enabled_) {
    if (HandleCameraControl(event)) {
      handled = true;
    }
  }

  // Handle selection last (only if no tool is active and selection is enabled)
  if (selection_enabled_) {
    if (HandleObjectSelection(event)) {
      return true;  // Consume selection events
    }
  }

  return handled;
}

bool SceneInputHandler::HandleKeyboardEvent(const InputEvent& event) {
  // Handle active tool input first (highest priority)
  if (auto active_tool = scene_manager_->GetActiveTool()) {
    if (active_tool->OnInputEvent(event)) {
      return true;  // Tool consumed the event
    }
  }

  // Handle keyboard shortcuts for built-in functionality
  if (event.GetType() == InputEventType::kKeyPress) {
    const auto& mods = event.GetModifiers();
    int key = event.GetKey();

    // Example: Delete key to delete selection
    if (key == 261 && selection_enabled_) {  // GLFW_KEY_DELETE
      // TODO: Implement delete selected objects
      return true;
    }

    // Example: Escape to clear selection
    if (key == 256 && selection_enabled_) {  // GLFW_KEY_ESCAPE
      scene_manager_->GetSelection().ClearSelection();
      return true;
    }
  }

  return false;
}

bool SceneInputHandler::HandleCameraControl(const InputEvent& event) {
  auto* camera_controller = scene_manager_->GetCameraController();
  if (!camera_controller) return false;

  switch (event.GetType()) {
    case InputEventType::kMousePress: {
      int button = event.GetMouseButton();
      if (IsCameraControlButton(button, event.GetModifiers())) {
        camera_active_ = true;
        active_camera_button_ = button;
        // Store mouse position (already in panel-local coordinates)
        last_mouse_pos_ = event.GetScreenPosition();
        return true;
      }
      break;
    }

    case InputEventType::kMouseRelease: {
      if (camera_active_ && event.GetMouseButton() == active_camera_button_) {
        camera_active_ = false;
        active_camera_button_ = -1;
        return true;
      }
      break;
    }

    case InputEventType::kMouseDrag:
    case InputEventType::kMouseMove: {
      if (camera_active_) {
        // Get current position (already in panel-local coordinates)
        glm::vec2 current_pos = event.GetScreenPosition();
        glm::vec2 delta = current_pos - last_mouse_pos_;
        last_mouse_pos_ = current_pos;
        
        // Use decoupled movement methods based on camera configuration
        MouseButton active_button = static_cast<MouseButton>(active_camera_button_);
        if (camera_config_.IsOrbitControl(active_button, {})) {
          // Explicit orbit movement with sensitivity
          camera_controller->ProcessOrbitMovement(
              delta.x * camera_config_.orbit_sensitivity, 
              delta.y * camera_config_.orbit_sensitivity);
        } else if (camera_config_.IsPanControl(active_button, {})) {
          // Explicit pan movement with sensitivity
          camera_controller->ProcessPanMovement(
              delta.x * camera_config_.pan_sensitivity, 
              delta.y * camera_config_.pan_sensitivity);
        }
        return true;
      }
      break;
    }

    case InputEventType::kMouseWheel: {
      if (camera_config_.enable_wheel_zoom) {
        glm::vec2 scroll_delta = event.GetDelta();
        camera_controller->ProcessMouseScroll(scroll_delta.y * camera_config_.zoom_sensitivity);
        return true;
      }
      break;
    }

    default:
      break;
  }

  return false;
}

bool SceneInputHandler::HandleObjectSelection(const InputEvent& event) {
  if (event.GetType() != InputEventType::kMousePress) return false;

  int button = event.GetMouseButton();
  if (!IsSelectionButton(button, event.GetModifiers())) return false;

  // Get mouse position (already converted to panel-local coordinates by GlScenePanel)
  glm::vec2 mouse_pos = event.GetScreenPosition();
  
  float x = mouse_pos.x;
  float y = mouse_pos.y;
  
  // Validate coordinates are within viewport bounds
  if (x < 0 || x >= viewport_width_ || y < 0 || y >= viewport_height_) {
    return false;
  }

  // Perform the actual selection
  auto selection_result = scene_manager_->Select(static_cast<int>(x), static_cast<int>(y));
  
  // Handle selection based on modifiers
  const auto& mods = event.GetModifiers();
  if (mods.ctrl) {
    // Add to selection
    // TODO: Implement additive selection
  } else if (mods.shift) {
    // Range selection or box selection start
    // TODO: Implement range/box selection
  } else {
    // Single selection (replace current selection)
    // This is handled by the selection manager in scene_manager_->Select()
  }

  return true;  // Consume selection events
}

bool SceneInputHandler::IsCameraControlButton(int button, const ModifierKeys& modifiers) const {
  MouseButton mouse_button = static_cast<MouseButton>(button);
  return camera_config_.IsCameraControl(mouse_button, modifiers);
}

bool SceneInputHandler::IsSelectionButton(int button, const ModifierKeys& modifiers) const {
  MouseButton mouse_button = static_cast<MouseButton>(button);
  return camera_config_.IsSelectionControl(mouse_button, modifiers);
}

glm::vec3 SceneInputHandler::ScreenToWorld(const glm::vec2& screen_pos, float depth) const {
  if (!scene_manager_) return glm::vec3(0);
  
  // TODO: Implement screen to world coordinate transformation
  // This would use the camera's projection and view matrices
  return glm::vec3(screen_pos.x, screen_pos.y, depth);
}

glm::vec2 SceneInputHandler::WorldToScreen(const glm::vec3& world_pos) const {
  if (!scene_manager_) return glm::vec2(0);
  
  // TODO: Implement world to screen coordinate transformation
  // This would use the camera's projection and view matrices
  return glm::vec2(world_pos.x, world_pos.y);
}

// Factory implementations
std::shared_ptr<SceneInputHandler> SceneInputHandlerFactory::CreateStandard(
    SceneManager* scene_manager, int priority) {
  auto handler = std::make_shared<SceneInputHandler>(scene_manager, priority);
  handler->SetCameraControlConfig(CameraControlConfig::ModelingSoftware());
  handler->SetCameraControlEnabled(true);
  handler->SetSelectionEnabled(true);
  return handler;
}

std::shared_ptr<SceneInputHandler> SceneInputHandlerFactory::CreateFPSStyle(
    SceneManager* scene_manager, int priority) {
  auto handler = std::make_shared<SceneInputHandler>(scene_manager, priority);
  handler->SetCameraControlConfig(CameraControlConfig::FPSStyle());
  handler->SetCameraControlEnabled(true);
  handler->SetSelectionEnabled(true);
  return handler;
}

std::shared_ptr<SceneInputHandler> SceneInputHandlerFactory::CreateWebViewer(
    SceneManager* scene_manager, int priority) {
  auto handler = std::make_shared<SceneInputHandler>(scene_manager, priority);
  handler->SetCameraControlConfig(CameraControlConfig::WebViewer());
  handler->SetCameraControlEnabled(true);
  handler->SetSelectionEnabled(false); // Web viewer typically disables selection
  return handler;
}

std::shared_ptr<SceneInputHandler> SceneInputHandlerFactory::CreateCustom(
    SceneManager* scene_manager, const CameraControlConfig& config, int priority) {
  auto handler = std::make_shared<SceneInputHandler>(scene_manager, priority);
  handler->SetCameraControlConfig(config);
  handler->SetCameraControlEnabled(config.enable_orbit || config.enable_pan);
  handler->SetSelectionEnabled(config.enable_selection);
  return handler;
}

std::shared_ptr<SceneInputHandler> SceneInputHandlerFactory::CreateCameraOnly(
    SceneManager* scene_manager, int priority) {
  auto handler = std::make_shared<SceneInputHandler>(scene_manager, priority);
  handler->SetCameraControlConfig(CameraControlConfig::ModelingSoftware());
  handler->SetCameraControlEnabled(true);
  handler->SetSelectionEnabled(false);
  return handler;
}

std::shared_ptr<SceneInputHandler> SceneInputHandlerFactory::CreateSelectionOnly(
    SceneManager* scene_manager, int priority) {
  auto handler = std::make_shared<SceneInputHandler>(scene_manager, priority);
  handler->SetCameraControlConfig(CameraControlConfig::ModelingSoftware());
  handler->SetCameraControlEnabled(false);
  handler->SetSelectionEnabled(true);
  return handler;
}

}  // namespace quickviz