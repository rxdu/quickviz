# Input Handling System for scene

This directory contains a comprehensive input handling system for the scene module, providing flexible and extensible user interaction capabilities.

## Overview

The input handling system is designed around the **InteractionMode** pattern, where different input behaviors (camera control, object selection, tools) are implemented as separate modes that can be switched between or stacked.

### Core Components

- **`InteractionMode`** - Base interface for all input behaviors
- **`InputContext`** - Shared context providing scene access and utilities  
- **`CameraInteractionMode`** - Camera navigation with multiple control schemes
- **`SelectionInteractionMode`** - Object selection with various selection modes
- **`GlInputHandler`** - Main orchestrator managing modes and input routing

## Quick Start

```cpp
#include "scene/gui/gl_input_handler.hpp"
#include "scene/input/camera_interaction_mode.hpp"
#include "scene/input/selection_interaction_mode.hpp"

// Create input handler
auto input_handler = std::make_unique<GlInputHandler>();

// Create interaction modes
auto camera_mode = std::make_unique<CameraInteractionMode>(
    CameraInteractionMode::ControlScheme::kOrbit);
auto selection_mode = std::make_unique<SelectionInteractionMode>(
    SelectionInteractionMode::SelectionMode::kSingle);

// Set up context
auto& context = input_handler->GetContext();
context.SetScene(scene_manager);
context.SetCamera(camera);
context.SetSelection(&selection_manager);
context.SetViewportSize(glm::ivec2(width, height));

// Register modes
input_handler->RegisterMode("camera", camera_mode.get());
input_handler->RegisterMode("selection", selection_mode.get());

// Set default mode
input_handler->SetMode(camera_mode.get());

// In your input event loop:
bool consumed = input_handler->HandleMousePress(button, glm::vec2(x, y));
input_handler->HandleMouseMove(glm::vec2(x, y));
input_handler->HandleMouseScroll(glm::vec2(dx, dy));
input_handler->HandleKeyPress(key);

// Update each frame
input_handler->Update(delta_time);

// Render overlays if needed
RenderContext render_ctx = context.CreateRenderContext();
input_handler->Render(render_ctx);
```

## Camera Control Modes

### Orbit Mode (Default)
- **Left drag**: Orbit around target point
- **Middle drag**: Pan view  
- **Scroll**: Zoom in/out

### First Person Mode
- **WASD**: Move forward/back/left/right
- **Mouse**: Look around
- **Q/E or Space/Ctrl**: Move up/down
- **Shift**: Sprint (2x speed)

### Top Down Mode  
- **Drag**: Pan view
- **Scroll**: Zoom in/out (adjusts height)

### Fly Mode
- **WASD**: Move in view direction
- **Q/E**: Move up/down in world space
- **Mouse drag**: Rotate view
- **Scroll**: Adjust movement speed

```cpp
// Configure camera mode
camera_mode->SetControlScheme(CameraInteractionMode::ControlScheme::kFirstPerson);
camera_mode->SetMouseSensitivity(2.0f);
camera_mode->SetKeyboardSpeed(10.0f);
camera_mode->SetInvertY(true);
```

## Selection Modes

### Single Selection
- **Click**: Select single object
- **Click empty space**: Clear selection

### Multiple Selection  
- **Ctrl+Click**: Add/remove from selection
- **Shift+Click**: Add to selection
- **Click without modifiers**: Replace selection

### Box Selection
- **Drag**: Create selection rectangle
- **Ctrl+Drag**: Add to selection
- **Shift+Drag**: Remove from selection

### Lasso Selection
- **Drag**: Draw freeform selection area
- **Ctrl+Drag**: Add to selection

```cpp
// Configure selection mode
selection_mode->SetSelectionMode(SelectionInteractionMode::SelectionMode::kBox);
selection_mode->SetHighlightOnHover(true);
selection_mode->SetSelectionRadius(10.0f);

// Set up callbacks
selection_mode->SetSelectionCallback([](const SelectionResult& result) {
    // Handle single selection
});
selection_mode->SetMultiSelectionCallback([](const MultiSelection& selection) {
    // Handle multi-selection
});
```

## Mode Management

### Basic Mode Switching
```cpp
// Switch between modes
input_handler->SetMode(camera_mode.get());
input_handler->SwitchToMode("selection");  // Using registered name

// Check current mode
if (input_handler->IsInMode("CameraMode")) {
    // Currently in camera mode
}
```

### Mode Stack (Temporary Modes)
```cpp
// Push temporary mode (e.g., hold Shift for selection)
if (shift_pressed) {
    input_handler->PushMode(selection_mode.get());
} else {
    input_handler->PopMode();  // Return to previous mode
}
```

### Mode Change Callbacks
```cpp
input_handler->SetModeChangeCallback([](InteractionMode* old_mode, InteractionMode* new_mode) {
    std::cout << "Mode changed from " << (old_mode ? old_mode->GetName() : "none") 
              << " to " << (new_mode ? new_mode->GetName() : "none") << std::endl;
});
```

## Input Context

The InputContext provides utilities for coordinate transformations and state access:

```cpp
auto& context = input_handler->GetContext();

// Coordinate transformations
glm::vec3 world_pos = context.ScreenToWorld(screen_pos, depth);
glm::vec3 screen_pos = context.WorldToScreen(world_pos);
Ray ray = context.ScreenPointToRay(screen_pos);

// State access
glm::vec2 mouse_pos = context.GetMousePosition();
glm::vec2 mouse_delta = context.GetMouseDelta();
ModifierKeys mods = context.GetModifiers();
bool in_viewport = context.IsInViewport(screen_pos);

// NDC conversions
glm::vec2 ndc = context.ScreenToNDC(screen_pos);
glm::vec2 screen = context.NDCToScreen(ndc);
```

## Custom Interaction Modes

Create custom interaction modes by inheriting from InteractionMode:

```cpp
class MeasureToolMode : public InteractionMode {
public:
    std::string GetName() const override { return "MeasureTool"; }
    
    bool OnMousePress(const MouseEvent& event, InputContext& context) override {
        if (event.button == MouseButton::kLeft) {
            // Start measurement
            auto ray = context.ScreenPointToRay(event.position);
            start_point_ = PerformRaycast(ray, context);
            return true;
        }
        return false;
    }
    
    void OnRender(const RenderContext& render_ctx) override {
        // Draw measurement visualization
        if (is_measuring_) {
            DrawLine(start_point_, end_point_);
            DrawText(FormatDistance(glm::distance(start_point_, end_point_)));
        }
    }
    
private:
    glm::vec3 start_point_, end_point_;
    bool is_measuring_ = false;
};
```

## Testing

The input system includes comprehensive unit tests:

```bash
# Run minimal unit tests (no GUI dependencies)
./bin/test_input_minimal

# Run integration demo
./bin/simple_input_demo

# Run via CMake
ctest -R test_input_minimal -V
```

## Architecture Notes

- **Event-driven**: Uses return values to indicate consumption for proper event propagation
- **Context-based**: No global state access; all context passed explicitly  
- **Extensible**: Virtual interfaces allow custom interaction modes
- **Testable**: Core functionality can be tested without OpenGL context
- **Composable**: Modes can be stacked and combined as needed

The system follows QuickViz's building blocks philosophy - providing generic, composable components that users can combine to build domain-specific applications.