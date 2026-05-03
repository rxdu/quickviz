/*
 * @file test_joystick_input.cpp
 * @date 2/13/25
 * @brief Test for modern gamepad enumeration system
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <cmath>
#include <vector>

#include "viewer/panel.hpp"
#include "viewer/viewer.hpp"
#include "viewer/input/gamepad_manager.hpp"

using namespace quickviz;

class GamepadPanel : public Panel {
 public:
  GamepadPanel(std::shared_ptr<Viewer> viewer)
      : Panel("GamepadPanel"), viewer_(viewer) {
    // This test demonstrates the UNIFIED INPUT SYSTEM:
    // - All gamepad input flows through InputEvent objects 
    // - The panel receives events via OnInputEvent() from InputEventHandler interface
    // - State is reconstructed from events, NOT polled directly from hardware
    // - This ensures consistent event-driven architecture across all input types
    
    auto& manager = GamepadManager::GetInstance();
    gamepads_ = manager.GetConnectedGamepads();
    selected_gamepad_id_ = -1;
    
    // Configure input policy to accept gamepad input
    InputPolicy policy = InputPolicy::AllowAll();
    policy.priority = 100; // High priority for gamepad test
    SetInputPolicy(policy);
    
    // Enable monitoring for real-time events
    manager.SetMonitoringEnabled(true);
    manager.SetConnectionCallback([this](const GamepadInfo& info, bool connected) {
      if (connected) {
        std::cout << "Gamepad connected: " << info.name << " (ID: " << info.id << ")" << std::endl;
      } else {
        std::cout << "Gamepad disconnected: " << info.name << " (ID: " << info.id << ")" << std::endl;
      }
      // Refresh gamepad list
      gamepads_ = GamepadManager::GetInstance().GetConnectedGamepads();
    });
  }

  std::string GetName() const override { return "GamepadPanel"; }

  // Override InputEventHandler methods to demonstrate unified input system
  bool OnInputEvent(const InputEvent& event) override {
    if (!ShouldProcessInput(event)) return false;
    
    // Handle gamepad events through unified system
    if (event.IsGamepadEvent() && event.GetGamepadId() == selected_gamepad_id_) {
      ProcessGamepadEvent(event);
      return true; // Consume the event
    }
    
    return false; // Don't consume non-gamepad events
  }
  
  int GetPriority() const override { 
    return GetInputPolicy().priority; 
  }

  void Draw() override {
    Begin();
    
    auto& manager = GamepadManager::GetInstance();
    
    // Refresh gamepad list
    gamepads_ = manager.GetConnectedGamepads();
    
    ImGui::Text("Number of Gamepad devices: %ld", gamepads_.size());

    if (gamepads_.size() > 0) {
      ImGui::Text("Available gamepads:");
      for (const auto& gamepad : gamepads_) {
        ImGui::Text("  ID: %d, Name: %s", gamepad.id, gamepad.name.c_str());
        ImGui::Text("    Axes: %d, Buttons: %d, Hats: %d", 
                   gamepad.num_axes, gamepad.num_buttons, gamepad.num_hats);
      }

      // Gamepad selection
      ImGui::Separator();
      ImGui::Text("Select gamepad for input monitoring:");
      
      for (const auto& gamepad : gamepads_) {
        if (ImGui::Button(("Monitor Gamepad " + std::to_string(gamepad.id)).c_str())) {
          selected_gamepad_id_ = gamepad.id;
        }
        ImGui::SameLine();
      }
      
      if (ImGui::Button("Stop Monitoring")) {
        selected_gamepad_id_ = -1;
      }

      // Display current gamepad state if monitoring
      if (selected_gamepad_id_ >= 0 && manager.IsGamepadConnected(selected_gamepad_id_)) {
        ImGui::Separator();
        ImGui::Text("Monitoring Gamepad %d:", selected_gamepad_id_);
        
        // Show unified input system events
        ImGui::Text("=== Unified Input System (Real-time from InputEvents) ===");
        ImGui::Text("Last Event: %s", last_event_description_.c_str());
        ImGui::Text("Events Received: %d", event_count_);
        
        ImGui::Separator();
        
        // Display unified system's tracked state
        ImGui::Text("Current State (built from InputEvents):");
        
        // Display axes from unified input events
        if (!unified_axis_values_.empty()) {
          ImGui::Text("Axes (%zu):", unified_axis_values_.size());
          ImGui::Indent();
          for (size_t i = 0; i < unified_axis_values_.size(); ++i) {
            ImGui::Text("Axis %zu:", i);
            ImGui::SameLine();
            
            // Progress bar for axis value (-1.0 to +1.0)
            float value = unified_axis_values_[i];
            float normalized_value = (value + 1.0f) / 2.0f; // Convert to 0.0-1.0
            char axis_label[32];
            snprintf(axis_label, sizeof(axis_label), "%+.3f", value);
            
            // Color based on value
            if (std::abs(value) > 0.1f) {
              ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.3f, 1.0f, 0.3f, 1.0f));
            }
            ImGui::ProgressBar(normalized_value, ImVec2(200, 0), axis_label);
            if (std::abs(value) > 0.1f) {
              ImGui::PopStyleColor();
            }
          }
          ImGui::Unindent();
        }
        
        // Display buttons from unified input events
        if (!unified_button_states_.empty()) {
          ImGui::Text("Buttons (%zu):", unified_button_states_.size());
          ImGui::Indent();
          for (size_t i = 0; i < unified_button_states_.size(); ++i) {
            bool is_pressed = unified_button_states_[i];
            if (is_pressed) {
              ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.3f, 0.3f, 1.0f)); // Red for pressed
            } else {
              ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.6f, 0.6f, 0.6f, 1.0f)); // Gray for released  
            }
            
            const char* button_state = is_pressed ? "PRESSED" : "released";
            ImGui::Text("  Button %zu: %s", i, button_state);
            ImGui::PopStyleColor();
          }
          ImGui::Unindent();
        }
        
        if (unified_axis_values_.empty() && unified_button_states_.empty()) {
          ImGui::Text("No input events received yet from unified system");
          ImGui::Text("(Input events are only generated when values change)");
        }
      }
    } else {
      ImGui::Text("No gamepads connected");
    }

    End();
  }

 private:
  void ProcessGamepadEvent(const InputEvent& event) {
    event_count_++;
    
    // Update tracked state from unified input events
    // This demonstrates that we're getting all gamepad data through the unified system
    switch (event.GetType()) {
      case InputEventType::kGamepadButtonPress:
      case InputEventType::kGamepadButtonRelease: {
        int button_id = event.GetButtonOrKey();
        bool is_pressed = (event.GetType() == InputEventType::kGamepadButtonPress);
        
        // Ensure button state vector is large enough
        if (button_id >= 0) {
          if (static_cast<size_t>(button_id) >= unified_button_states_.size()) {
            unified_button_states_.resize(button_id + 1, false);
          }
          unified_button_states_[button_id] = is_pressed;
        }
        
        char event_desc[256];
        snprintf(event_desc, sizeof(event_desc), 
                "Button %d %s (gamepad %d)", 
                button_id, is_pressed ? "PRESSED" : "RELEASED", event.GetGamepadId());
        last_event_description_ = event_desc;
        std::cout << "[InputEvent] " << event_desc << std::endl;
        break;
      }
      
      case InputEventType::kGamepadAxisMove: {
        int axis_id = event.GetAxisIndex();
        float value = event.GetAxisValue();
        
        // Ensure axis state vector is large enough
        if (axis_id >= 0) {
          if (static_cast<size_t>(axis_id) >= unified_axis_values_.size()) {
            unified_axis_values_.resize(axis_id + 1, 0.0f);
          }
          unified_axis_values_[axis_id] = value;
        }
        
        char event_desc[256];
        snprintf(event_desc, sizeof(event_desc), 
                "Axis %d = %+.3f (gamepad %d)", 
                axis_id, value, event.GetGamepadId());
        last_event_description_ = event_desc;
        // Don't spam console with axis movements
        if (std::abs(value) > 0.1f) {  // Only log significant movements
          std::cout << "[InputEvent] " << event_desc << std::endl;
        }
        break;
      }
      
      default: {
        char event_desc[256];
        snprintf(event_desc, sizeof(event_desc), 
                "Unknown gamepad event (gamepad %d)", event.GetGamepadId());
        last_event_description_ = event_desc;
        break;
      }
    }
  }

  std::shared_ptr<Viewer> viewer_;
  std::vector<GamepadInfo> gamepads_;
  int selected_gamepad_id_;
  
  // Unified input system tracking
  std::string last_event_description_ = "No events yet";
  int event_count_ = 0;
  
  // State reconstructed from unified input events
  std::vector<float> unified_axis_values_;
  std::vector<bool> unified_button_states_;
};

int main() {
  try {
    auto viewer = std::make_shared<Viewer>("Gamepad Input Test", 800, 600);
    auto gamepad_panel = std::make_shared<GamepadPanel>(viewer);
    viewer->AddSceneObject(gamepad_panel);
    viewer->Show();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}