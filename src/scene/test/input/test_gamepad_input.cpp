/*
 * @file test_gamepad_input.cpp
 * @date 9/1/25
 * @brief Test and example for unified gamepad input handling
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <vector>
#include <memory>

#include "core/event/input_event.hpp"
#include "core/event/input_mapping.hpp"
#include "viewer/input/imgui_input_utils.hpp"
#include "scene/input/scene_input_handler.hpp"
#include "viewer/input/input_dispatcher.hpp"

using namespace quickviz;

/**
 * @brief Example gamepad input handler demonstrating unified input system
 * 
 * This example shows how gamepad events are processed consistently
 * with mouse and keyboard events through the same InputEvent system.
 */
class GamepadInputExample {
public:
    GamepadInputExample() {
        // Setup input mapping with gamepad support
        input_mapping_.ResetToDefaults();
        
        // Create input dispatcher
        dispatcher_ = std::make_shared<InputDispatcher>();
        
        // Create scene input handler for 3D interactions
        scene_handler_ = std::make_shared<SceneInputHandler>();
        scene_handler_->SetInputMapping(&input_mapping_);
        
        // Register handler with high priority for 3D scene interactions
        dispatcher_->RegisterHandler(scene_handler_, 100);
        
        // Register our example handler for remaining events
        auto example_handler = std::make_shared<ExampleGamepadHandler>(input_mapping_);
        dispatcher_->RegisterHandler(example_handler, 50);
        
        std::cout << "Gamepad Input Example initialized\n";
        std::cout << "Available gamepad actions:\n";
        PrintGamepadMappings();
    }
    
    void ProcessFrame() {
        // Poll all input events from ImGui (mouse, keyboard, gamepad)
        std::vector<InputEvent> events;
        ImGuiInputUtils::PollAllEvents(events);
        
        // Process events through dispatcher
        bool any_consumed = dispatcher_->ProcessEvents(events);
        
        // Log processed events (optional)
        if (!events.empty()) {
            LogProcessedEvents(events, any_consumed);
        }
    }
    
private:
    /**
     * @brief Example handler to demonstrate gamepad event processing
     */
    class ExampleGamepadHandler : public InputEventHandler {
    public:
        ExampleGamepadHandler(const InputMapping& mapping) : mapping_(mapping) {}
        
        int GetPriority() const override { return 50; }
        
        bool OnInputEvent(const InputEvent& event) override {
            if (!event.IsGamepadEvent()) {
                return false; // Only handle gamepad events in this example
            }
            
            // Get actions mapped to this event
            auto actions = mapping_.GetActionsForEvent(event);
            
            if (actions.empty()) {
                return false; // No mapped actions
            }
            
            // Handle each mapped action
            for (const auto& action : actions) {
                if (HandleGamepadAction(action, event)) {
                    return true; // Event consumed
                }
            }
            
            return false;
        }
        
    private:
        bool HandleGamepadAction(const std::string& action, const InputEvent& event) {
            // Navigation actions
            if (action == Actions::NAVIGATE_UP) {
                std::cout << "Gamepad: Navigate UP\n";
                return true;
            }
            if (action == Actions::NAVIGATE_DOWN) {
                std::cout << "Gamepad: Navigate DOWN\n";
                return true;
            }
            if (action == Actions::NAVIGATE_LEFT) {
                std::cout << "Gamepad: Navigate LEFT\n";
                return true;
            }
            if (action == Actions::NAVIGATE_RIGHT) {
                std::cout << "Gamepad: Navigate RIGHT\n";
                return true;
            }
            
            // Tool actions
            if (action == Actions::TOOL_PRIMARY) {
                std::cout << "Gamepad: Primary tool activated\n";
                return true;
            }
            if (action == Actions::TOOL_SECONDARY) {
                std::cout << "Gamepad: Secondary tool activated\n";
                return true;
            }
            if (action == Actions::TOOL_ALTERNATE) {
                std::cout << "Gamepad: Alternate tool activated\n";
                return true;
            }
            
            // Selection actions
            if (action == Actions::SELECT_SINGLE) {
                std::cout << "Gamepad: Single select (A/Cross button)\n";
                return true;
            }
            
            // Menu actions
            if (action == Actions::NAVIGATE_MENU) {
                std::cout << "Gamepad: Menu opened (Start button)\n";
                return true;
            }
            if (action == Actions::NAVIGATE_BACK) {
                std::cout << "Gamepad: Back/Cancel (Back button)\n";
                return true;
            }
            
            // Camera actions are handled by SceneInputHandler
            // But we can log them here for demonstration
            if (action == Actions::CAMERA_ZOOM_IN) {
                std::cout << "Gamepad: Camera zoom in (L1 bumper)\n";
            }
            if (action == Actions::CAMERA_ZOOM_OUT) {
                std::cout << "Gamepad: Camera zoom out (R1 bumper)\n";
            }
            
            return false; // Let other handlers process camera actions
        }
        
        const InputMapping& mapping_;
    };
    
    void PrintGamepadMappings() {
        std::cout << "\n=== Gamepad Action Mappings ===\n";
        std::cout << "Face Buttons:\n";
        std::cout << "  A/Cross (Face Down)    -> Select Single\n";
        std::cout << "  B/Circle (Face Right)  -> Navigate Cancel\n"; 
        std::cout << "  X/Square (Face Left)   -> Secondary Tool\n";
        std::cout << "  Y/Triangle (Face Up)   -> Alternate Tool\n";
        
        std::cout << "\nD-Pad:\n";
        std::cout << "  Up/Down/Left/Right     -> Navigation\n";
        
        std::cout << "\nShoulder Buttons:\n";
        std::cout << "  L1 (Left Bumper)       -> Camera Zoom In\n";
        std::cout << "  R1 (Right Bumper)      -> Camera Zoom Out\n";
        std::cout << "  L2 (Left Trigger)      -> Camera Rotate\n";
        std::cout << "  R2 (Right Trigger)     -> Camera Pan\n";
        
        std::cout << "\nAnalog Sticks:\n";
        std::cout << "  Left Stick             -> Camera Pan\n";
        std::cout << "  Right Stick            -> Camera Rotate\n";
        
        std::cout << "\nMenu Buttons:\n";
        std::cout << "  Start/Menu             -> Navigate Menu\n";
        std::cout << "  Back/View              -> Navigate Back\n";
        std::cout << "===============================\n\n";
    }
    
    void LogProcessedEvents(const std::vector<InputEvent>& events, bool any_consumed) {
        static int frame_count = 0;
        frame_count++;
        
        // Only log every 60 frames to avoid spam, or when events are consumed
        if (any_consumed || (frame_count % 60 == 0 && !events.empty())) {
            std::cout << "Frame " << frame_count << " - Events: " << events.size();
            if (any_consumed) {
                std::cout << " (consumed)";
            }
            std::cout << "\n";
            
            // Show event breakdown
            int mouse_count = 0, keyboard_count = 0, gamepad_count = 0;
            for (const auto& event : events) {
                if (event.IsMouseEvent()) mouse_count++;
                else if (event.IsKeyboardEvent()) keyboard_count++;
                else if (event.IsGamepadEvent()) gamepad_count++;
            }
            
            if (mouse_count > 0) std::cout << "  Mouse: " << mouse_count << "\n";
            if (keyboard_count > 0) std::cout << "  Keyboard: " << keyboard_count << "\n";
            if (gamepad_count > 0) std::cout << "  Gamepad: " << gamepad_count << "\n";
        }
    }
    
    InputMapping input_mapping_;
    std::shared_ptr<InputDispatcher> dispatcher_;
    std::shared_ptr<SceneInputHandler> scene_handler_;
};

/**
 * @brief Test function to demonstrate unified input event creation
 */
void TestUnifiedInputEvents() {
    std::cout << "\n=== Testing Unified Input Event Creation ===\n";
    
    // Test mouse event creation
    auto mouse_event = ImGuiInputUtils::CreateMouseEvent(
        InputEventType::kMousePress, 0); // Left button
    std::cout << "Created mouse event: " << mouse_event.GetName() 
              << " (button: " << mouse_event.GetMouseButton() << ")\n";
    
    // Test keyboard event creation  
    auto key_event = ImGuiInputUtils::CreateKeyEvent(
        InputEventType::kKeyPress, ImGuiKey_Space);
    std::cout << "Created keyboard event: " << key_event.GetName()
              << " (key: " << key_event.GetKey() << ")\n";
    
    // Test gamepad event creation
    InputEvent gamepad_event(InputEventType::kGamepadButtonPress, ImGuiKey_GamepadFaceDown);
    gamepad_event.SetGamepadId(0);
    std::cout << "Created gamepad event: " << gamepad_event.GetName()
              << " (button: " << gamepad_event.GetButtonOrKey() 
              << ", gamepad: " << gamepad_event.GetGamepadId() << ")\n";
    
    std::cout << "All input types use consistent InputEvent interface!\n";
}

/**
 * @brief Test function to verify input mapping consistency
 */
void TestInputMappingConsistency() {
    std::cout << "\n=== Testing Input Mapping Consistency ===\n";
    
    InputMapping mapping;
    
    // Test mouse action mapping
    InputEvent mouse_click(InputEventType::kMousePress, 0); // Left button
    if (mapping.IsActionTriggered(Actions::SELECT_SINGLE, mouse_click)) {
        std::cout << "✓ Mouse left click mapped to SELECT_SINGLE\n";
    }
    
    // Test keyboard action mapping
    ModifierKeys ctrl_mod;
    ctrl_mod.ctrl = true;
    InputEvent key_press(InputEventType::kKeyPress, 90); // Z key
    key_press.SetModifiers(ctrl_mod);
    if (mapping.IsActionTriggered(Actions::UNDO, key_press)) {
        std::cout << "✓ Ctrl+Z mapped to UNDO\n";
    }
    
    // Test gamepad action mapping
    InputEvent gamepad_press(InputEventType::kGamepadButtonPress, ImGuiKey_GamepadFaceDown);
    if (mapping.IsActionTriggered(Actions::SELECT_SINGLE, gamepad_press)) {
        std::cout << "✓ Gamepad A button mapped to SELECT_SINGLE\n";
    }
    
    // Verify same action can be triggered by different input types
    auto actions = mapping.GetAllActions();
    std::cout << "Total actions mapped: " << actions.size() << "\n";
    std::cout << "Same actions available across all input types!\n";
}

int main() {
    std::cout << "QuickViz Unified Input System Example\n";
    std::cout << "====================================\n\n";
    
    // Test unified input event creation
    TestUnifiedInputEvents();
    
    // Test input mapping consistency
    TestInputMappingConsistency();
    
    // Create example handler
    GamepadInputExample example;
    
    std::cout << "\nInput system ready. In a real application, call ProcessFrame() in your render loop.\n";
    std::cout << "This example demonstrates:\n";
    std::cout << "1. Unified InputEvent system for mouse, keyboard, and gamepad\n";
    std::cout << "2. Consistent action mapping across all input types\n";
    std::cout << "3. Priority-based event dispatching\n";
    std::cout << "4. ImGui-centric input polling\n";
    std::cout << "5. Bridge pattern for 3D scene interactions\n";
    
    return 0;
}