/*
 * @file input_mapping.hpp
 * @date 8/30/25
 * @brief Configurable input mapping system for QuickViz
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_INPUT_MAPPING_HPP
#define QUICKVIZ_INPUT_MAPPING_HPP

#include <string>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <sstream>

#include "core/event/input_event.hpp"

namespace quickviz {

// Predefined action constants
namespace Actions {
constexpr const char* SELECT_SINGLE = "select_single";
constexpr const char* SELECT_ADD = "select_add";
constexpr const char* SELECT_TOGGLE = "select_toggle";
constexpr const char* SELECT_BOX = "select_box";
constexpr const char* SELECT_LASSO = "select_lasso";
constexpr const char* CAMERA_ROTATE = "camera_rotate";
constexpr const char* CAMERA_PAN = "camera_pan";
constexpr const char* CAMERA_ZOOM = "camera_zoom";
constexpr const char* CLEAR_SELECTION = "clear_selection";
constexpr const char* DELETE_SELECTED = "delete_selected";
constexpr const char* UNDO = "undo";
constexpr const char* REDO = "redo";
constexpr const char* COPY = "copy";
constexpr const char* PASTE = "paste";
constexpr const char* CUT = "cut";
}  // namespace Actions

class InputMapping {
 public:
  struct Binding {
    int trigger;                 // Button or key code
    ModifierKeys modifiers;      // Required modifier keys
    InputEventType event_type;  // Type of event (press, release, etc.)

    bool operator==(const Binding& other) const {
      return trigger == other.trigger && modifiers == other.modifiers &&
             event_type == other.event_type;
    }
  };

  InputMapping() { SetupDefaultMappings(); }

  // Map actions to input combinations
  void MapMouseAction(const std::string& action, int button,
                      const ModifierKeys& modifiers = ModifierKeys(),
                      InputEventType type = InputEventType::kMousePress) {
    Binding binding;
    binding.trigger = button;
    binding.modifiers = modifiers;
    binding.event_type = type;
    action_bindings_[action].push_back(binding);
  }

  void MapKeyAction(const std::string& action, int key,
                    const ModifierKeys& modifiers = ModifierKeys(),
                    InputEventType type = InputEventType::kKeyPress) {
    Binding binding;
    binding.trigger = key;
    binding.modifiers = modifiers;
    binding.event_type = type;
    action_bindings_[action].push_back(binding);
  }

  // Remove specific binding
  void UnmapAction(const std::string& action) {
    action_bindings_.erase(action);
  }

  void RemoveBinding(const std::string& action, const Binding& binding) {
    auto it = action_bindings_.find(action);
    if (it != action_bindings_.end()) {
      auto& bindings = it->second;
      bindings.erase(std::remove(bindings.begin(), bindings.end(), binding),
                     bindings.end());
      if (bindings.empty()) {
        action_bindings_.erase(it);
      }
    }
  }

  // Query actions
  bool IsActionTriggered(const std::string& action,
                         const InputEvent& event) const {
    auto it = action_bindings_.find(action);
    if (it == action_bindings_.end()) return false;

    for (const auto& binding : it->second) {
      if (binding.event_type != event.GetType()) continue;
      if (binding.trigger != event.GetButtonOrKey()) continue;
      if (binding.modifiers != event.GetModifiers()) continue;
      return true;
    }
    return false;
  }

  std::vector<std::string> GetActionsForEvent(const InputEvent& event) const {
    std::vector<std::string> actions;
    for (const auto& [action, bindings] : action_bindings_) {
      for (const auto& binding : bindings) {
        if (binding.event_type == event.GetType() &&
            binding.trigger == event.GetButtonOrKey() &&
            binding.modifiers == event.GetModifiers()) {
          actions.push_back(action);
        }
      }
    }
    return actions;
  }

  std::string GetPrimaryActionForEvent(const InputEvent& event) const {
    auto actions = GetActionsForEvent(event);
    return actions.empty() ? "" : actions.front();
  }

  // Get all bindings for an action
  std::vector<Binding> GetBindings(const std::string& action) const {
    auto it = action_bindings_.find(action);
    return (it != action_bindings_.end()) ? it->second
                                           : std::vector<Binding>();
  }

  // Clear all mappings
  void Clear() { action_bindings_.clear(); }

  // Reset to default mappings
  void ResetToDefaults() {
    Clear();
    SetupDefaultMappings();
  }

  // Serialization (simple text format)
  void SaveToFile(const std::string& path) const {
    std::ofstream file(path);
    if (!file.is_open()) return;

    for (const auto& [action, bindings] : action_bindings_) {
      for (const auto& binding : bindings) {
        file << action << " ";
        file << static_cast<int>(binding.event_type) << " ";
        file << binding.trigger << " ";
        file << (binding.modifiers.ctrl ? 1 : 0) << " ";
        file << (binding.modifiers.shift ? 1 : 0) << " ";
        file << (binding.modifiers.alt ? 1 : 0) << " ";
        file << (binding.modifiers.super ? 1 : 0) << "\n";
      }
    }
  }

  void LoadFromFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) return;

    Clear();
    std::string line;
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      std::string action;
      int event_type, trigger;
      int ctrl, shift, alt, super;

      if (iss >> action >> event_type >> trigger >> ctrl >> shift >> alt >>
          super) {
        Binding binding;
        binding.event_type = static_cast<InputEventType>(event_type);
        binding.trigger = trigger;
        binding.modifiers.ctrl = ctrl != 0;
        binding.modifiers.shift = shift != 0;
        binding.modifiers.alt = alt != 0;
        binding.modifiers.super = super != 0;
        action_bindings_[action].push_back(binding);
      }
    }
  }

  // Get all registered actions
  std::vector<std::string> GetAllActions() const {
    std::vector<std::string> actions;
    for (const auto& [action, _] : action_bindings_) {
      actions.push_back(action);
    }
    return actions;
  }

 private:
  void SetupDefaultMappings() {
    // Mouse button constants (from imview/input/mouse.hpp)
    const int kLeft = 0;
    const int kRight = 1; 
    const int kMiddle = 2;

    // Selection actions
    MapMouseAction(Actions::SELECT_SINGLE, kLeft);

    ModifierKeys ctrl_mod;
    ctrl_mod.ctrl = true;
    MapMouseAction(Actions::SELECT_ADD, kLeft, ctrl_mod);

    ModifierKeys shift_mod;
    shift_mod.shift = true;
    MapMouseAction(Actions::SELECT_BOX, kLeft, shift_mod);

    // Camera actions
    MapMouseAction(Actions::CAMERA_ROTATE, kRight);
    MapMouseAction(Actions::CAMERA_PAN, kMiddle);

    // Keyboard shortcuts (using GLFW key codes - should be defined elsewhere)
    // These are placeholder values - actual key codes depend on the windowing
    // system
    const int KEY_DELETE = 261;  // GLFW_KEY_DELETE
    const int KEY_ESCAPE = 256;  // GLFW_KEY_ESCAPE
    const int KEY_Z = 90;         // GLFW_KEY_Z
    const int KEY_Y = 89;         // GLFW_KEY_Y
    const int KEY_C = 67;         // GLFW_KEY_C
    const int KEY_V = 86;         // GLFW_KEY_V
    const int KEY_X = 88;         // GLFW_KEY_X

    MapKeyAction(Actions::DELETE_SELECTED, KEY_DELETE);
    MapKeyAction(Actions::CLEAR_SELECTION, KEY_ESCAPE);

    MapKeyAction(Actions::UNDO, KEY_Z, ctrl_mod);
    
    ModifierKeys ctrl_shift_mod;
    ctrl_shift_mod.ctrl = true;
    ctrl_shift_mod.shift = true;
    MapKeyAction(Actions::REDO, KEY_Z, ctrl_shift_mod);

    MapKeyAction(Actions::COPY, KEY_C, ctrl_mod);
    MapKeyAction(Actions::PASTE, KEY_V, ctrl_mod);
    MapKeyAction(Actions::CUT, KEY_X, ctrl_mod);
  }

  std::unordered_map<std::string, std::vector<Binding>> action_bindings_;
};

}  // namespace quickviz

#endif  // QUICKVIZ_INPUT_MAPPING_HPP