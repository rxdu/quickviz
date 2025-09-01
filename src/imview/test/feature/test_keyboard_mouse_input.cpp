/*
 * @file test_keyboard_mouse_input.cpp
 * @date 9/1/25
 * @brief Test for unified keyboard and mouse input system
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <cmath>
#include <vector>
#include <sstream>
#include <iomanip>
#include <set>

#include "imview/panel.hpp"
#include "imview/viewer.hpp"

using namespace quickviz;

class KeyboardMousePanel : public Panel {
 public:
  KeyboardMousePanel(std::shared_ptr<Viewer> viewer)
      : Panel("KeyboardMousePanel"), viewer_(viewer) {
    // This test demonstrates the UNIFIED INPUT SYSTEM for keyboard and mouse:
    // - All keyboard/mouse input flows through InputEvent objects 
    // - The panel receives events via OnInputEvent() from InputEventHandler interface
    // - State is reconstructed from events, NOT polled directly from ImGui/GLFW
    // - This ensures consistent event-driven architecture across all input types
    // - Same system used for gamepad input (see test_joystick_input)
    
    // Configure input policy to accept keyboard and mouse input
    InputPolicy policy = InputPolicy::AllowAll();
    policy.priority = 100; // High priority for input test
    SetInputPolicy(policy);
  }

  std::string GetName() const override { return "KeyboardMousePanel"; }

  // Override InputEventHandler methods to demonstrate unified input system
  bool OnInputEvent(const InputEvent& event) override {
    if (!ShouldProcessInput(event)) return false;
    
    // Process keyboard and mouse events through unified system
    if (event.IsMouseEvent()) {
      ProcessMouseEvent(event);
      return true; // Consume the event
    }
    
    if (event.IsKeyboardEvent()) {
      ProcessKeyboardEvent(event);
      return true; // Consume the event
    }
    
    return false; // Don't consume other events
  }
  
  int GetPriority() const override { 
    return GetInputPolicy().priority; 
  }

  void Draw() override {
    Begin();
    
    ImGui::Text("=== Unified Input System Test (Keyboard & Mouse) ===");
    ImGui::Separator();
    
    // Show input policy status
    auto policy = GetInputPolicy();
    ImGui::Text("Input Policy: Priority=%d, Mouse=%s, Keyboard=%s", 
                policy.priority,
                policy.accept_mouse ? "ACCEPT" : "REJECT",
                policy.accept_keyboard ? "ACCEPT" : "REJECT");
    
    // Show ImGui keyboard capture status
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureKeyboard) {
      ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.7f, 0.3f, 1.0f)); // Orange
      ImGui::Text("Status: ImGui is capturing keyboard (UI has focus)");
      ImGui::PopStyleColor();
    } else {
      ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.3f, 1.0f, 0.3f, 1.0f)); // Green
      ImGui::Text("Status: Application keyboard input enabled");
      ImGui::PopStyleColor();
    }
    
    ImGui::Separator();
    ImGui::Text("Events Received: %d", event_count_);
    ImGui::Text("Last Event: %s", last_event_description_.c_str());
    
    // Mouse state section
    ImGui::Separator();
    ImGui::Text("=== Mouse State (from InputEvents) ===");
    ImGui::Text("Position: (%.1f, %.1f)", mouse_pos_.x, mouse_pos_.y);
    ImGui::Text("Delta: (%.1f, %.1f)", mouse_delta_.x, mouse_delta_.y);
    ImGui::Text("Wheel: %.2f", mouse_wheel_);
    
    // Mouse buttons
    ImGui::Text("Buttons:");
    ImGui::Indent();
    for (int i = 0; i < 3; ++i) {
      const char* button_names[] = {"Left", "Right", "Middle"};
      bool is_pressed = (i < static_cast<int>(mouse_buttons_.size())) && mouse_buttons_[i];
      
      if (is_pressed) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.3f, 0.3f, 1.0f)); // Red
        ImGui::Text("%s: PRESSED", button_names[i]);
        ImGui::PopStyleColor();
      } else {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.6f, 0.6f, 1.0f)); // Gray
        ImGui::Text("%s: released", button_names[i]);
        ImGui::PopStyleColor();
      }
    }
    ImGui::Unindent();
    
    // Keyboard state section
    ImGui::Separator();
    ImGui::Text("=== Keyboard State (from InputEvents) ===");
    
    // Modifier keys
    ImGui::Text("Modifiers: %s%s%s%s",
                last_modifiers_.ctrl ? "[CTRL] " : "",
                last_modifiers_.shift ? "[SHIFT] " : "",
                last_modifiers_.alt ? "[ALT] " : "",
                last_modifiers_.super ? "[SUPER] " : "");
    
    // Currently pressed keys
    ImGui::Text("Pressed Keys:");
    ImGui::Indent();
    if (pressed_keys_.empty()) {
      ImGui::Text("(none)");
    } else {
      std::stringstream ss;
      for (int key : pressed_keys_) {
        ss << GetKeyName(key) << " ";
      }
      ImGui::Text("%s", ss.str().c_str());
    }
    ImGui::Unindent();
    
    // Recent key events
    ImGui::Separator();
    ImGui::Text("=== Recent Events (last 10) ===");
    ImGui::BeginChild("EventLog", ImVec2(0, 150), true);
    for (auto it = event_log_.rbegin(); it != event_log_.rend(); ++it) {
      ImGui::Text("%s", it->c_str());
    }
    ImGui::EndChild();
    
    // Test instructions
    ImGui::Separator();
    ImGui::Text("=== Instructions ===");
    ImGui::BulletText("Move mouse to see position updates");
    ImGui::BulletText("Click mouse buttons to test press/release");
    ImGui::BulletText("Scroll mouse wheel to test wheel events");
    ImGui::BulletText("Press keyboard keys to test key events");
    ImGui::BulletText("Hold Ctrl/Shift/Alt to test modifiers");
    
    ImGui::Separator();
    ImGui::Text("=== Keyboard Input Solutions ===");
    ImGui::BulletText("Method 1: Click outside this window to enable keyboard");
    ImGui::BulletText("Method 2: Press Ctrl+Shift+K to bypass ImGui capture");
    ImGui::Text("(ImGui normally captures keyboard when UI has focus)");
    ImGui::BulletText("All events flow through unified InputEvent system");
    
    End();
  }

 private:
  void ProcessMouseEvent(const InputEvent& event) {
    event_count_++;
    
    // Update mouse state from unified input events
    switch (event.GetType()) {
      case InputEventType::kMousePress: {
        int button = event.GetMouseButton();
        if (button >= 0 && button < 3) {
          if (mouse_buttons_.size() <= static_cast<size_t>(button)) {
            mouse_buttons_.resize(button + 1, false);
          }
          mouse_buttons_[button] = true;
        }
        
        std::string desc = "Mouse " + GetMouseButtonName(button) + " PRESSED";
        last_event_description_ = desc;
        AddToEventLog(desc);
        break;
      }
      
      case InputEventType::kMouseRelease: {
        int button = event.GetMouseButton();
        if (button >= 0 && button < 3) {
          if (mouse_buttons_.size() <= static_cast<size_t>(button)) {
            mouse_buttons_.resize(button + 1, false);
          }
          mouse_buttons_[button] = false;
        }
        
        std::string desc = "Mouse " + GetMouseButtonName(button) + " RELEASED";
        last_event_description_ = desc;
        AddToEventLog(desc);
        break;
      }
      
      case InputEventType::kMouseMove: {
        mouse_pos_ = event.GetScreenPosition();
        mouse_delta_ = event.GetDelta();
        
        char desc[256];
        snprintf(desc, sizeof(desc), "Mouse MOVE to (%.1f, %.1f)", 
                 mouse_pos_.x, mouse_pos_.y);
        last_event_description_ = desc;
        // Don't log every move event to avoid spam
        break;
      }
      
      case InputEventType::kMouseDrag: {
        mouse_pos_ = event.GetScreenPosition();
        mouse_delta_ = event.GetDelta();
        
        char desc[256];
        snprintf(desc, sizeof(desc), "Mouse DRAG to (%.1f, %.1f)", 
                 mouse_pos_.x, mouse_pos_.y);
        last_event_description_ = desc;
        // Log drag events at reduced rate
        if (event_count_ % 10 == 0) {
          AddToEventLog(desc);
        }
        break;
      }
      
      case InputEventType::kMouseWheel: {
        auto delta = event.GetDelta();
        mouse_wheel_ += delta.y; // Vertical scroll
        
        char desc[256];
        snprintf(desc, sizeof(desc), "Mouse WHEEL %.2f (total: %.2f)", 
                 delta.y, mouse_wheel_);
        last_event_description_ = desc;
        AddToEventLog(desc);
        break;
      }
      
      default:
        break;
    }
    
    // Always update modifiers from mouse events
    last_modifiers_ = event.GetModifiers();
  }
  
  void ProcessKeyboardEvent(const InputEvent& event) {
    event_count_++;
    
    int key = event.GetKey();
    last_modifiers_ = event.GetModifiers();
    
    switch (event.GetType()) {
      case InputEventType::kKeyPress: {
        // Add to pressed keys set
        pressed_keys_.insert(key);
        
        std::string desc = "Key " + GetKeyName(key) + " PRESSED";
        if (last_modifiers_.ctrl || last_modifiers_.shift || 
            last_modifiers_.alt || last_modifiers_.super) {
          desc += " (with modifiers)";
        }
        last_event_description_ = desc;
        AddToEventLog(desc);
        break;
      }
      
      case InputEventType::kKeyRelease: {
        // Remove from pressed keys set
        pressed_keys_.erase(key);
        
        std::string desc = "Key " + GetKeyName(key) + " RELEASED";
        last_event_description_ = desc;
        AddToEventLog(desc);
        break;
      }
      
      default:
        break;
    }
  }
  
  void AddToEventLog(const std::string& event) {
    event_log_.push_back(event);
    if (event_log_.size() > 10) {
      event_log_.erase(event_log_.begin());
    }
  }
  
  std::string GetMouseButtonName(int button) const {
    switch (button) {
      case 0: return "LEFT";
      case 1: return "RIGHT";
      case 2: return "MIDDLE";
      default: return "BUTTON" + std::to_string(button);
    }
  }
  
  std::string GetKeyName(int key) const {
    // Map ImGui key codes to readable names
    // This is a simplified version - you could expand this
    if (key >= ImGuiKey_A && key <= ImGuiKey_Z) {
      return std::string(1, 'A' + (key - ImGuiKey_A));
    }
    if (key >= ImGuiKey_0 && key <= ImGuiKey_9) {
      return std::string(1, '0' + (key - ImGuiKey_0));
    }
    
    switch (key) {
      case ImGuiKey_Space: return "SPACE";
      case ImGuiKey_Enter: return "ENTER";
      case ImGuiKey_Escape: return "ESCAPE";
      case ImGuiKey_Tab: return "TAB";
      case ImGuiKey_Delete: return "DELETE";
      case ImGuiKey_Backspace: return "BACKSPACE";
      case ImGuiKey_LeftArrow: return "LEFT";
      case ImGuiKey_RightArrow: return "RIGHT";
      case ImGuiKey_UpArrow: return "UP";
      case ImGuiKey_DownArrow: return "DOWN";
      case ImGuiKey_LeftCtrl: return "LCTRL";
      case ImGuiKey_RightCtrl: return "RCTRL";
      case ImGuiKey_LeftShift: return "LSHIFT";
      case ImGuiKey_RightShift: return "RSHIFT";
      case ImGuiKey_LeftAlt: return "LALT";
      case ImGuiKey_RightAlt: return "RALT";
      default: return "KEY" + std::to_string(key);
    }
  }

  std::shared_ptr<Viewer> viewer_;
  
  // Unified input system tracking
  std::string last_event_description_ = "No events yet";
  int event_count_ = 0;
  std::vector<std::string> event_log_;
  
  // Mouse state reconstructed from unified input events
  glm::vec2 mouse_pos_{0, 0};
  glm::vec2 mouse_delta_{0, 0};
  float mouse_wheel_ = 0.0f;
  std::vector<bool> mouse_buttons_;
  
  // Keyboard state reconstructed from unified input events
  std::set<int> pressed_keys_;
  ModifierKeys last_modifiers_;
};

int main() {
  try {
    auto viewer = std::make_shared<Viewer>("Keyboard & Mouse Input Test", 1024, 768);
    auto input_panel = std::make_shared<KeyboardMousePanel>(viewer);
    viewer->AddSceneObject(input_panel);
    viewer->Show();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}