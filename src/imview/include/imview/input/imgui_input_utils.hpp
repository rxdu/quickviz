/*
 * @file imgui_input_utils.hpp
 * @date 9/1/25
 * @brief ImGui-centric input utilities for creating input events
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef IMVIEW_IMGUI_INPUT_UTILS_HPP
#define IMVIEW_IMGUI_INPUT_UTILS_HPP

#include "imgui.h"

#include "core/event/input_event.hpp"
#include "core/event/input_mapping.hpp"

#include "imview/input/input_types.hpp"

namespace quickviz {
/**
 * @brief Utility class for creating InputEvents from ImGui state
 *
 * This class provides static methods to convert ImGui input state
 * into our standardized InputEvent objects for consistent handling.
 */
class ImGuiInputUtils {
 public:
  /**
   * @brief Create mouse event from current ImGui state
   * @param type Type of mouse event
   * @param button Mouse button (use MouseButton enum values)
   * @return InputEvent with current mouse position and modifiers
   */
  static InputEvent CreateMouseEvent(InputEventType type, int button = -1);

  /**
   * @brief Create keyboard event from current ImGui state
   * @param type Type of keyboard event
   * @param key Key code (ImGui key values)
   * @return InputEvent with current modifiers
   */
  static InputEvent CreateKeyEvent(InputEventType type, int key);

  /**
   * @brief Get current modifier keys from ImGui
   * @return ModifierKeys structure with current state
   */
  static ModifierKeys GetCurrentModifiers();

  /**
   * @brief Convert ImGui mouse position to content-relative coordinates
   * @param content_offset Offset of content region from window origin
   * @return Mouse position relative to content area
   */
  static glm::vec2 GetContentRelativeMousePos(
      const glm::vec2& content_offset = glm::vec2(0));

  /**
   * @brief Check if mouse is over current ImGui window content
   * @return true if mouse is hovering over window content area
   */
  static bool IsMouseOverContent();

  /**
   * @brief Get mouse delta from ImGui
   * @return Mouse movement delta since last frame
   */
  static glm::vec2 GetMouseDelta();

  /**
   * @brief Check for mouse click events and create corresponding InputEvents
   * @param events Output vector to store generated events
   */
  static void PollMouseEvents(std::vector<InputEvent>& events);

  /**
   * @brief Check for keyboard events and create corresponding InputEvents
   * @param events Output vector to store generated events
   */
  static void PollKeyboardEvents(std::vector<InputEvent>& events);

  /**
   * @brief Check for gamepad/joystick events and create corresponding
   * InputEvents
   * @param events Output vector to store generated events
   */
  static void PollGamepadEvents(std::vector<InputEvent>& events);

  /**
   * @brief Check for all input events in one call
   * @param events Output vector to store all generated events
   */
  static void PollAllEvents(std::vector<InputEvent>& events);

  /**
   * @brief Helper to check if ImGui wants to capture input
   * @return true if ImGui should handle mouse input
   */
  static bool ShouldCaptureMouseInput();

  /**
   * @brief Helper to check if ImGui wants to capture keyboard
   * @return true if ImGui should handle keyboard input
   */
  static bool ShouldCaptureKeyboardInput();

  /**
   * @brief Helper to check if ImGui should handle gamepad input
   * @return true if ImGui should handle gamepad input
   */
  static bool ShouldCaptureGamepadInput();
};

/**
 * @brief RAII class for scoped input polling within a panel
 *
 * Usage:
 *   ScopedInputPoller poller;
 *   if (poller.HasEvents()) {
 *     for (const auto& event : poller.GetEvents()) {
 *       // Handle event
 *     }
 *   }
 */
class ScopedInputPoller {
 public:
  ScopedInputPoller();
  ~ScopedInputPoller() = default;

  bool HasEvents() const { return !events_.empty(); }
  const std::vector<InputEvent>& GetEvents() const { return events_; }

  // Get specific event types
  std::vector<InputEvent> GetMouseEvents() const;
  std::vector<InputEvent> GetKeyboardEvents() const;

 private:
  std::vector<InputEvent> events_;
};

}  // namespace quickviz

#endif  // IMVIEW_IMGUI_INPUT_UTILS_HPP