# Virtual Scene Architecture Design

*Created: August 27, 2025*  
*Purpose: Document architectural decisions for virtual scene layer implementation*

## Overview

QuickViz is evolving from a pure rendering library to an interactive 3D visualization library. The virtual scene architecture provides clean separation between application logic, interaction management, and rendering backend while maintaining QuickViz's role as a reusable library component.

## Architecture Goals

### **Design Philosophy**
- **QuickViz remains a library** - no application-specific business logic
- **Clean separation of concerns** - UI, interaction, and rendering are separate
- **Event-driven architecture** - loose coupling between layers
- **Professional interaction model** - similar to game engines and CAD software
- **Incremental implementation** - working code at each step

### **Target Use Case**
Primary target: **Robotics map editing applications** requiring:
- Interactive waypoint placement and editing
- Path/route visualization and editing  
- Point cloud selection and cropping
- Region/zone annotation
- Measurement and analysis tools

## Proposed Architecture

### **Data Flow Overview**
```
Application Logic ← Events ← VirtualScene ← SceneViewPanel ← ImGui ← GLFW
Application Logic → VirtualObjects → IRenderBackend → gldraw → OpenGL
```

### **Module Responsibilities**

**Application Layer**:
- Business logic (waypoint semantics, route planning, etc.)
- Data persistence and serialization
- Application-specific workflows

**vscene Module** (future):
- Virtual object management
- Interaction state tracking
- Event dispatching
- Scene graph operations

**gldraw Module**:
- Pure OpenGL rendering backend
- GPU resource management
- Primitive implementations

**imview Module**:
- UI framework integration
- Panel management and layout

## Core Class Design

### **Virtual Scene Layer**

```cpp
class VirtualScene {
    // Object management
    void AddObject(const std::string& id, std::unique_ptr<VirtualObject> obj);
    VirtualObject* Pick(float screen_x, float screen_y);
    
    // State management  
    void SetSelected(const std::string& id, bool selected);
    std::vector<std::string> GetSelectedIds();
    
    // Rendering coordination
    void Update(float delta_time);
    void Render();
    void SetRenderBackend(std::unique_ptr<IRenderBackend> backend);
};

class VirtualObject {
    // Transform and state
    glm::mat4 transform_;
    bool visible_, selected_, hovered_;
    
    // Interface
    virtual BoundingBox GetBounds() const = 0;
    virtual bool HitTest(const Ray& ray, float& distance) const = 0;
    virtual void UpdateBackend(IRenderBackend* backend) = 0;
    
    // Events
    std::function<void(VirtualObject*)> OnClick;
    std::function<void(VirtualObject*, glm::vec3)> OnDrag;
};
```

### **Backend Interface**

```cpp
class IRenderBackend {
    virtual void CreateObject(const std::string& id, const std::string& type) = 0;
    virtual void UpdateObject(const std::string& id, const ObjectData& data) = 0;
    virtual std::string Pick(float screen_x, float screen_y) = 0;
    virtual void Render() = 0;
};

class GlDrawBackend : public IRenderBackend {
    std::unique_ptr<GlSceneManager> scene_manager_;
    // Implements interface using existing gldraw functionality
};
```

### **Event System**

```cpp
enum class VirtualEventType {
    ObjectClicked, ObjectDragged, SelectionChanged,
    BackgroundClicked, ObjectHovered, // ... 
};

struct VirtualEvent {
    VirtualEventType type;
    std::string object_id;
    glm::vec2 screen_pos;
    glm::vec3 world_pos;
    std::any data;
};

class EventDispatcher {
    void Subscribe(VirtualEventType type, std::function<void(const VirtualEvent&)> handler);
    void Dispatch(const VirtualEvent& event);
};
```

## SceneViewPanel Separation

### **Current Problem**
```cpp
// Current problematic design
class GlSceneManager : public Panel {
    void Draw() override;      // Mixes ImGui UI with OpenGL rendering
    void SelectObjectAt();     // Interaction logic in rendering class
};
```

### **Target Solution**
```cpp
// Clean separation
class GlSceneManager {
    void RenderToFramebuffer();  // Pure rendering, no UI knowledge
    std::string Pick(float x, float y);  // Backend service
};

class SceneViewPanel : public Panel {
    GlSceneManager* scene_manager_;
    EventDispatcher* event_dispatcher_;
    
    void Draw() override;         // Only handles ImGui integration
    void HandleInput();           // Converts ImGui events to application events
};
```

### **Event Flow Implementation**

**Input Processing**:
```cpp
void SceneViewPanel::HandleInput() {
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        // 1. Convert screen coordinates
        float local_x = mouse_pos.x - window_pos.x - content_min.x;
        float local_y = mouse_pos.y - window_pos.y - content_min.y;
        
        // 2. Pick object via backend
        VirtualObject* picked = virtual_scene_->Pick(local_x, local_y);
        
        // 3. Update scene state
        if (picked) {
            virtual_scene_->SetSelected(picked->GetId(), true);
        }
        
        // 4. Dispatch event to application
        VirtualEvent event{
            .type = VirtualEventType::ObjectClicked,
            .object_id = picked ? picked->GetId() : "",
            .screen_pos = {local_x, local_y}
        };
        event_dispatcher_->Dispatch(event);
    }
}
```

**Application Response**:
```cpp
// Robotics map editor
void MapEditor::Initialize() {
    dispatcher->Subscribe(VirtualEventType::ObjectClicked, 
        [this](const VirtualEvent& e) {
            if (!e.object_id.empty()) {
                ShowWaypointProperties(e.object_id);
            }
        });
        
    dispatcher->Subscribe(VirtualEventType::BackgroundClicked,
        [this](const VirtualEvent& e) {
            CreateWaypointAt(e.world_pos);
        });
}
```

## Implementation Strategy

### **Phase 1: SceneViewPanel Separation** (Current Priority)
**Goal**: Clean separation between UI integration and rendering

**Approach**: Incremental refactoring with minimal risk
- Create SceneViewPanel in gldraw module (for testability)
- Extract ImGui code from GlSceneManager 
- Test with existing applications one by one
- Keep all rendering logic unchanged

**Module Placement Decision**:
- **Start in gldraw** - easier testing and visualization
- **Move to vscene later** - when virtual scene layer is ready
- **No new dependencies** - everything self-contained initially

**Benefits**:
- Visual debugging and immediate feedback
- All test infrastructure already exists  
- Easy rollback if issues arise
- Validates separation concept

### **Phase 2: Virtual Scene Layer** (Future)
**Goal**: Professional interaction and object management

**Components**:
- Virtual object hierarchy (VirtualSphere, VirtualMesh, etc.)
- Scene management and state tracking
- Event system for application communication
- Backend abstraction for rendering

### **Phase 3: Advanced Interactions** (Future)
**Goal**: Production-ready manipulation tools

**Features**:
- ImGuizmo integration for transform gizmos
- Multi-selection and manipulation
- Constraint systems (grid snapping, axis locking)
- Context menus and tool modes

## Integration Patterns

### **For Library Users**

**Basic Usage**:
```cpp
// Create scene view
auto scene_panel = std::make_shared<SceneViewPanel>("3D View");
viewer.AddSceneObject(scene_panel);

// Add objects  
auto sphere = std::make_unique<VirtualSphere>();
sphere->SetPosition({0, 0, 1});
sphere->OnClick = [](VirtualObject* obj) { 
    std::cout << "Clicked sphere" << std::endl; 
};
scene_panel->GetVirtualScene()->AddObject("sphere1", std::move(sphere));
```

**Event-Driven Usage**:
```cpp
// Subscribe to events
auto dispatcher = scene_panel->GetEventDispatcher();
dispatcher->Subscribe(VirtualEventType::ObjectClicked, 
    [this](const VirtualEvent& e) {
        HandleObjectClick(e.object_id, e.screen_pos);
    });
```

### **Backwards Compatibility**

Existing applications using GlSceneManager directly can:
1. **Continue using GlSceneManager** - still works for pure rendering
2. **Migrate to SceneViewPanel** - drop-in replacement with more features
3. **Upgrade incrementally** - no forced migrations

## Benefits Summary

### **For QuickViz Library**
- **Clean architecture** following industry standards
- **Testable components** with clear responsibilities  
- **Extensible design** easy to add new object types
- **Performance optimizations** possible at virtual layer
- **Multiple backend support** (OpenGL, Vulkan, etc.)

### **For Applications**
- **Professional interaction model** users expect
- **Event-driven integration** with loose coupling
- **Easy to extend** with application-specific objects
- **Consistent behavior** across all object types
- **Rich manipulation tools** (gizmos, constraints, etc.)

### **For Development**
- **Incremental implementation** with working code at each step
- **Easy testing** and visual debugging
- **Clear migration path** from current to target architecture
- **Risk mitigation** through small, reversible changes

## Conclusion

The virtual scene architecture transforms QuickViz into a professional interactive 3D visualization library while maintaining its core identity as a reusable component. The incremental implementation strategy ensures working code at each step while building toward a robust, extensible architecture suitable for demanding applications like robotics map editors.

The SceneViewPanel separation is the critical first step that validates the approach and provides immediate architectural benefits with minimal risk.