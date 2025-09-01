/*
 * @file scene_input_handler.cpp
 * @date 9/1/25
 * @brief Implementation of 3D scene input handler bridge
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "gldraw/input/scene_input_handler.hpp"
#include "gldraw/gl_scene_manager.hpp"
#include "imview/input/mouse.hpp"
#include "core/event/input_mapping.hpp"

namespace quickviz {

SceneInputHandler::SceneInputHandler(GlSceneManager* scene_manager, int priority)
    : scene_manager_(scene_manager), priority_(priority) {
}

bool SceneInputHandler::OnInputEvent(const InputEvent& event) {
  if (!enabled_ || !scene_manager_) return false;

  if (event.IsMouseEvent()) {
    return HandleMouseEvent(event);
  } else if (event.IsKeyboardEvent()) {
    return HandleKeyboardEvent(event);
  }

  return false;
}

bool SceneInputHandler::HandleMouseEvent(const InputEvent& event) {
  bool handled = false;

  // Handle camera control first (lower priority in terms of consumption)
  if (camera_control_enabled_) {
    if (HandleCameraControl(event)) {
      handled = true;
    }
  }

  // Handle selection (higher priority - can consume events)
  if (selection_enabled_) {
    if (HandleObjectSelection(event)) {
      return true;  // Consume selection events
    }
  }

  return handled;
}

bool SceneInputHandler::HandleKeyboardEvent(const InputEvent& event) {
  // Handle keyboard shortcuts
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
        last_mouse_pos_ = event.GetScreenPosition();
        camera_controller->SetActiveMouseButton(static_cast<MouseButton>(button));
        return true;
      }
      break;
    }

    case InputEventType::kMouseRelease: {
      if (camera_active_ && event.GetMouseButton() == active_camera_button_) {
        camera_active_ = false;
        active_camera_button_ = -1;
        camera_controller->SetActiveMouseButton(MouseButton::kNone);
        return true;
      }
      break;
    }

    case InputEventType::kMouseDrag:
    case InputEventType::kMouseMove: {
      if (camera_active_) {
        glm::vec2 current_pos = event.GetScreenPosition();
        glm::vec2 delta = current_pos - last_mouse_pos_;
        last_mouse_pos_ = current_pos;
        
        // Update camera based on active button
        if (active_camera_button_ == MouseButton::kRight) {
          camera_controller->ProcessMouseMovement(delta.x, delta.y);
        } else if (active_camera_button_ == MouseButton::kMiddle) {
          // Pan camera - for now, we'll use mouse movement
          // TODO: Implement proper pan functionality if needed
          camera_controller->ProcessMouseMovement(delta.x, delta.y);
        }
        return true;
      }
      break;
    }

    case InputEventType::kMouseWheel: {
      glm::vec2 scroll_delta = event.GetDelta();
      camera_controller->ProcessMouseScroll(scroll_delta.y);
      return true;
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

  // Perform selection
  glm::vec2 mouse_pos = event.GetScreenPosition();
  
  // Convert to viewport coordinates if needed
  float x = mouse_pos.x;
  float y = mouse_pos.y;

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
  // Right mouse button for orbit/rotate
  if (button == MouseButton::kRight && modifiers.IsEmpty()) return true;
  
  // Middle mouse button for pan
  if (button == MouseButton::kMiddle && modifiers.IsEmpty()) return true;
  
  return false;
}

bool SceneInputHandler::IsSelectionButton(int button, const ModifierKeys& modifiers) const {
  // Left mouse button for selection (with or without modifiers)
  return button == MouseButton::kLeft;
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
    GlSceneManager* scene_manager, int priority) {
  auto handler = std::make_shared<SceneInputHandler>(scene_manager, priority);
  handler->SetCameraControlEnabled(true);
  handler->SetSelectionEnabled(true);
  return handler;
}

std::shared_ptr<SceneInputHandler> SceneInputHandlerFactory::CreateCameraOnly(
    GlSceneManager* scene_manager, int priority) {
  auto handler = std::make_shared<SceneInputHandler>(scene_manager, priority);
  handler->SetCameraControlEnabled(true);
  handler->SetSelectionEnabled(false);
  return handler;
}

std::shared_ptr<SceneInputHandler> SceneInputHandlerFactory::CreateSelectionOnly(
    GlSceneManager* scene_manager, int priority) {
  auto handler = std::make_shared<SceneInputHandler>(scene_manager, priority);
  handler->SetCameraControlEnabled(false);
  handler->SetSelectionEnabled(true);
  return handler;
}

}  // namespace quickviz