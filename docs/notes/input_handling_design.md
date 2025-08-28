# Input Handling System Design

*Date: August 2025*  
*Author: Ruixiang Du*  
*Status: Design Proposal*

## Overview

This document outlines the design for an improved input handling system for QuickViz, addressing the current limitations in mouse/keyboard event processing and selection operations. The new system provides flexible, extensible input management suitable for complex robotics visualization applications.

## Current System Analysis

### Issues with Current Implementation

1. **Hardcoded Input Bindings**
   - Selection only triggers on left-click (gl_scene_panel.cpp:177)
   - No keyboard modifier support (Ctrl, Shift, Alt)
   - Cannot customize bindings per application

2. **Limited Callback System**
   - Single SelectionCallback in SelectionManager
   - No event priorities or consumption mechanism
   - Cannot chain multiple handlers

3. **Input Conflicts**
   - Camera control and selection compete for mouse input
   - No clear priority system
   - No way to temporarily disable certain inputs

4. **Missing Context**
   - Callbacks lack information about triggering input
   - No application-specific data passing
   - Limited selection action types (only select, no hover/preview)

## Proposed Architecture

### Layer 1: Event-Driven Input System

```cpp
// Core input event representation
class InputEvent {
public:
    enum Type { 
        kMousePress, 
        kMouseRelease, 
        kMouseMove,
        kMouseDrag,
        kMouseWheel,
        kKeyPress, 
        kKeyRelease 
    };
    
    struct ModifierKeys {
        bool ctrl : 1;
        bool shift : 1;
        bool alt : 1;
        bool super : 1;
    };
    
    Type type;
    int button_or_key;     // Mouse button or key code
    ModifierKeys modifiers;
    glm::vec2 screen_pos;  // Current mouse position
    glm::vec2 delta;       // Movement delta (for drag/wheel)
    float timestamp;       // Event timestamp
    bool consumed = false; // Allow event consumption
    void* user_data = nullptr;
};

// Priority-based event handler interface
class InputHandler {
public:
    virtual ~InputHandler() = default;
    virtual int GetPriority() const = 0;  // Higher = processed first
    virtual bool OnInputEvent(const InputEvent& event) = 0;  // Return true to consume
    virtual std::string GetName() const = 0;
};

// Central input dispatcher
class InputDispatcher {
public:
    void RegisterHandler(std::shared_ptr<InputHandler> handler);
    void UnregisterHandler(const std::string& name);
    void DispatchEvent(const InputEvent& event);
    void SetEnabled(bool enabled);
    
private:
    std::vector<std::shared_ptr<InputHandler>> handlers_; // Sorted by priority
    bool enabled_ = true;
};
```

### Layer 2: Configurable Input Mapping

```cpp
// Action-based input mapping
class InputMapping {
public:
    // Define actions with input combinations
    void MapMouseAction(const std::string& action, 
                       int button, 
                       InputEvent::ModifierKeys modifiers);
    void MapKeyAction(const std::string& action, 
                     int key, 
                     InputEvent::ModifierKeys modifiers);
    
    // Query actions
    bool IsActionTriggered(const std::string& action, const InputEvent& event) const;
    std::string GetActionForEvent(const InputEvent& event) const;
    
    // Serialization for saving/loading configurations
    void SaveToFile(const std::string& path) const;
    void LoadFromFile(const std::string& path);
    
private:
    struct Binding {
        int trigger;  // Button or key
        InputEvent::ModifierKeys modifiers;
    };
    std::unordered_map<std::string, std::vector<Binding>> action_bindings_;
};

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
}
```

### Layer 3: Enhanced Selection System

```cpp
// Rich selection event with context
struct SelectionEvent {
    enum Action {
        kSelect,      // New selection
        kAdd,         // Add to selection
        kRemove,      // Remove from selection
        kToggle,      // Toggle selection state
        kHover,       // Mouse hovering (preview)
        kDragStart,   // Begin dragging selection
        kDragEnd      // Complete dragging
    };
    
    SelectionResult result;     // What was selected
    Action action;              // Type of selection action
    InputEvent input;           // Original input event
    glm::vec3 ray_origin;       // 3D ray for selection
    glm::vec3 ray_direction;    
    void* app_context;          // Application-specific data
};

// Flexible selection handler system
class SelectionManager {
public:
    using SelectionHandler = std::function<bool(const SelectionEvent&)>;
    using SelectionFilter = std::function<bool(const SelectionResult&)>;
    
    // Handler management with priorities
    void AddHandler(const std::string& name, 
                   SelectionHandler handler, 
                   int priority = 0);
    void RemoveHandler(const std::string& name);
    void ClearHandlers();
    
    // Pre/post processing hooks
    void SetPreSelectionFilter(SelectionFilter filter);
    void SetPostSelectionAction(std::function<void(const SelectionEvent&)> action);
    
    // Selection state
    void SetSelectionMode(SelectionMode mode);
    void SetHighlightMode(HighlightMode mode);
    
private:
    std::multimap<int, std::pair<std::string, SelectionHandler>> handlers_;
    SelectionFilter pre_filter_;
    std::function<void(const SelectionEvent&)> post_action_;
};
```

### Layer 4: Selection Tools Framework

```cpp
// Base class for selection tools
class SelectionTool : public InputHandler {
public:
    enum class State { kIdle, kActive, kDragging };
    
    virtual void OnActivate() {}
    virtual void OnDeactivate() {}
    virtual void OnUpdate(float delta_time) {}
    virtual void OnRender() {}  // Visual feedback rendering
    
    State GetState() const { return state_; }
    
protected:
    State state_ = State::kIdle;
    glm::vec2 start_pos_;
    glm::vec2 current_pos_;
};

// Point selection tool
class PointSelectionTool : public SelectionTool {
public:
    bool OnInputEvent(const InputEvent& event) override;
    void OnRender() override;  // Draw crosshair
    
private:
    float selection_radius_ = 3.0f;
};

// Box selection tool
class BoxSelectionTool : public SelectionTool {
public:
    bool OnInputEvent(const InputEvent& event) override;
    void OnRender() override;  // Draw selection rectangle
    
private:
    bool draw_filled_ = false;
    glm::vec4 box_color_ = glm::vec4(1, 1, 0, 0.3f);
};

// Lasso selection tool
class LassoSelectionTool : public SelectionTool {
public:
    bool OnInputEvent(const InputEvent& event) override;
    void OnRender() override;  // Draw lasso polygon
    
private:
    std::vector<glm::vec2> polygon_points_;
    float close_threshold_ = 10.0f;
};

// Tool manager
class SelectionToolManager {
public:
    void SetActiveTool(std::shared_ptr<SelectionTool> tool);
    std::shared_ptr<SelectionTool> GetActiveTool() const;
    void RegisterTool(const std::string& name, std::shared_ptr<SelectionTool> tool);
    void SwitchTool(const std::string& name);
    
private:
    std::shared_ptr<SelectionTool> active_tool_;
    std::unordered_map<std::string, std::shared_ptr<SelectionTool>> tools_;
};
```

### Layer 5: Visual Feedback System

```cpp
// Selection visualization configuration
class SelectionVisualizer {
public:
    enum class Style {
        kOutline,        // Border outline
        kGlow,          // Glowing effect
        kColorTint,     // Color overlay
        kBoundingBox,   // 3D bounding box
        kWireframe,     // Wireframe overlay
        kCustomShader   // User-provided shader
    };
    
    struct VisualizationParams {
        Style style = Style::kOutline;
        glm::vec3 color = glm::vec3(1, 1, 0);
        float intensity = 1.0f;
        float line_width = 2.0f;
        bool animate = false;
        float animation_speed = 1.0f;
    };
    
    // Configure visualization
    void SetSelectionStyle(const VisualizationParams& params);
    void SetHoverStyle(const VisualizationParams& params);
    void SetMultiSelectionStyle(const VisualizationParams& params);
    
    // Enable/disable features
    void EnableHoverHighlight(bool enable);
    void EnableSelectionPersistence(bool enable);
    void EnableAnimations(bool enable);
    
    // Apply visualization to objects
    void ApplyToSelection(const SelectionResult& selection);
    void ClearVisualization();
    
private:
    VisualizationParams selection_params_;
    VisualizationParams hover_params_;
    VisualizationParams multi_params_;
    bool hover_enabled_ = true;
    bool animations_enabled_ = true;
};
```

## Integration Example

```cpp
class RoboticsVisualizationApp {
public:
    void InitializeInput() {
        // 1. Configure input mappings
        auto& input_map = scene_->GetInputMapping();
        
        // Selection actions
        input_map.MapMouseAction(Actions::SELECT_SINGLE, 
                                MouseButton::kLeft, {});
        input_map.MapMouseAction(Actions::SELECT_ADD, 
                                MouseButton::kLeft, {.ctrl = true});
        input_map.MapMouseAction(Actions::SELECT_BOX, 
                                MouseButton::kLeft, {.shift = true});
        
        // Camera actions
        input_map.MapMouseAction(Actions::CAMERA_ROTATE, 
                                MouseButton::kRight, {});
        input_map.MapMouseAction(Actions::CAMERA_PAN, 
                                MouseButton::kMiddle, {});
        
        // 2. Register selection handlers
        scene_->GetSelection().AddHandler("main_handler",
            [this](const SelectionEvent& e) {
                return this->OnSelectionEvent(e);
            }, 
            priority: 100
        );
        
        // 3. Configure visual feedback
        SelectionVisualizer::VisualizationParams params;
        params.style = SelectionVisualizer::Style::kGlow;
        params.color = glm::vec3(0, 1, 0);
        params.animate = true;
        scene_->GetVisualizer().SetSelectionStyle(params);
        
        // 4. Setup selection tools
        auto tool_mgr = scene_->GetToolManager();
        tool_mgr->RegisterTool("point", std::make_shared<PointSelectionTool>());
        tool_mgr->RegisterTool("box", std::make_shared<BoxSelectionTool>());
        tool_mgr->RegisterTool("lasso", std::make_shared<LassoSelectionTool>());
        tool_mgr->SetActiveTool("point");
    }
    
    bool OnSelectionEvent(const SelectionEvent& event) {
        switch (event.action) {
            case SelectionEvent::kSelect:
                selected_objects_.clear();
                selected_objects_.push_back(event.result);
                UpdateUI();
                return true;
                
            case SelectionEvent::kAdd:
                selected_objects_.push_back(event.result);
                UpdateUI();
                return true;
                
            case SelectionEvent::kHover:
                ShowTooltip(event.result);
                return false;  // Don't consume hover events
                
            default:
                return false;
        }
    }
};
```

## Implementation Phases

### Phase 1: Core Infrastructure (Week 1)
- [ ] Implement InputEvent and InputDispatcher
- [ ] Create InputHandler base class
- [ ] Integrate with existing GlScenePanel::HandleInput()
- [ ] Unit tests for event system

### Phase 2: Input Mapping (Week 1-2)
- [ ] Implement InputMapping class
- [ ] Create action constants
- [ ] Add configuration file support
- [ ] Update GlScenePanel to use mappings

### Phase 3: Selection Enhancement (Week 2)
- [ ] Extend SelectionManager with new handler system
- [ ] Implement SelectionEvent structure
- [ ] Add pre/post processing hooks
- [ ] Update existing selection code

### Phase 4: Selection Tools (Week 3)
- [ ] Create SelectionTool base class
- [ ] Implement PointSelectionTool
- [ ] Implement BoxSelectionTool
- [ ] Add SelectionToolManager

### Phase 5: Visual Feedback (Week 3-4)
- [ ] Design SelectionVisualizer interface
- [ ] Implement basic visualization styles
- [ ] Add animation support
- [ ] Integrate with layer system

### Phase 6: Testing & Polish (Week 4)
- [ ] Comprehensive unit tests
- [ ] Integration tests with sample apps
- [ ] Performance profiling
- [ ] Documentation and examples

## Benefits

1. **Flexibility**: Applications can customize all input behaviors without modifying library code
2. **Maintainability**: Clear separation of concerns between input, selection, and rendering
3. **Extensibility**: Easy to add new tools, selection modes, and visualizations
4. **Performance**: Event consumption prevents unnecessary processing
5. **User Experience**: Rich visual feedback and customizable controls
6. **Reusability**: Input system can be used for non-selection interactions

## Backward Compatibility

The new system will be implemented alongside the existing one with a compatibility layer:

```cpp
// Legacy API preserved
scene->Select(x, y);  // Maps to new system internally

// New API available
scene->GetInputDispatcher().DispatchEvent(event);
```

Applications can gradually migrate to the new system while maintaining existing functionality.

## Open Questions

1. Should we support gesture recognition (pinch, swipe)?
2. How to handle touch input for future tablet support?
3. Should input recordings/playback be supported for testing?
4. Integration with ImGui's input system?

## References

- Unity Input System: https://docs.unity3d.com/Packages/com.unity.inputsystem@1.0/
- Unreal Engine Input: https://docs.unrealengine.com/4.27/en-US/InteractiveExperiences/Input/
- Qt Event System: https://doc.qt.io/qt-5/eventsandfilters.html
- Dear ImGui Input: https://github.com/ocornut/imgui/blob/master/imgui.h
