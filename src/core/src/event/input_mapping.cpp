/*
 * @file input_mapping.cpp
 * @date 9/1/25
 * @brief Implementation of input mapping system
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "core/event/input_mapping.hpp"

// Include ImGui for gamepad constants in implementation only
#include "imgui.h"

namespace quickviz {

void InputMapping::SetupDefaultGamepadMappings() {
    // Gamepad mappings (using ImGui gamepad key constants)
    // Face buttons
    MapGamepadAction(Actions::SELECT_SINGLE, ImGuiKey_GamepadFaceDown);      // A/Cross
    MapGamepadAction(Actions::NAVIGATE_CANCEL, ImGuiKey_GamepadFaceRight);   // B/Circle
    MapGamepadAction(Actions::TOOL_SECONDARY, ImGuiKey_GamepadFaceLeft);     // X/Square  
    MapGamepadAction(Actions::TOOL_ALTERNATE, ImGuiKey_GamepadFaceUp);       // Y/Triangle

    // D-pad navigation
    MapGamepadAction(Actions::NAVIGATE_UP, ImGuiKey_GamepadDpadUp);
    MapGamepadAction(Actions::NAVIGATE_DOWN, ImGuiKey_GamepadDpadDown);
    MapGamepadAction(Actions::NAVIGATE_LEFT, ImGuiKey_GamepadDpadLeft);
    MapGamepadAction(Actions::NAVIGATE_RIGHT, ImGuiKey_GamepadDpadRight);

    // Shoulder buttons for camera control
    MapGamepadAction(Actions::CAMERA_ZOOM_IN, ImGuiKey_GamepadL1);           // Left bumper
    MapGamepadAction(Actions::CAMERA_ZOOM_OUT, ImGuiKey_GamepadR1);          // Right bumper
    MapGamepadAction(Actions::CAMERA_ROTATE, ImGuiKey_GamepadL2);            // Left trigger
    MapGamepadAction(Actions::CAMERA_PAN, ImGuiKey_GamepadR2);               // Right trigger

    // Menu buttons
    MapGamepadAction(Actions::NAVIGATE_MENU, ImGuiKey_GamepadStart);         // Start/Menu
    MapGamepadAction(Actions::NAVIGATE_BACK, ImGuiKey_GamepadBack);          // Back/View

    // Analog sticks for camera control (treated as directional buttons)
    MapGamepadAction(Actions::CAMERA_ROTATE, ImGuiKey_GamepadRStickLeft);
    MapGamepadAction(Actions::CAMERA_ROTATE, ImGuiKey_GamepadRStickRight);
    MapGamepadAction(Actions::CAMERA_ROTATE, ImGuiKey_GamepadRStickUp);
    MapGamepadAction(Actions::CAMERA_ROTATE, ImGuiKey_GamepadRStickDown);
    
    MapGamepadAction(Actions::CAMERA_PAN, ImGuiKey_GamepadLStickLeft);
    MapGamepadAction(Actions::CAMERA_PAN, ImGuiKey_GamepadLStickRight);
    MapGamepadAction(Actions::CAMERA_PAN, ImGuiKey_GamepadLStickUp);
    MapGamepadAction(Actions::CAMERA_PAN, ImGuiKey_GamepadLStickDown);
}

}  // namespace quickviz