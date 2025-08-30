# Virtual Scene (vscene) Interface Design

*Created: August 27, 2025*  
*Purpose: Document interface design and workflow for virtual scene layer*

## Overview

The vscene module provides a high-level interactive 3D scene management system built on top of the existing gldraw rendering system. It introduces application-level semantics, event-driven interaction, and clean separation between application logic and rendering.

## Core Design Principles

### 1. **Application-Level Semantics**
Objects represent meaningful application concepts (waypoints, targets, obstacles) rather than just geometric primitives.

```cpp
// Before: Low-level rendering primitives
auto sphere = std::make_unique<Sphere>(0.5f);
scene_manager->AddOpenGLObject("sphere1", std::move(sphere));

// After: Application-level virtual objects  
auto waypoint = CreateVirtualSphere("waypoint_001", 0.5f);
waypoint->SetPosition(glm::vec3(-2.0f, 0.0f, 0.0f));
scene_panel->AddObject("waypoint_001", std::move(waypoint));
```

### 2. **Event-Driven Interaction**
Clean separation between interaction events and application responses.

```cpp
// Object-level callbacks
waypoint->OnClick = [](VirtualObject* obj, glm::vec2 screen_pos, glm::vec3 world_pos) {
    ShowWaypointProperties(obj->GetId());
};

// Scene-level event subscriptions
event_dispatcher->Subscribe(VirtualEventType::BackgroundClicked,
    [](const VirtualEvent& e) {
        if (e.ctrl_pressed) {
            CreateWaypointAt(e.world_pos);
        }
    });
```

### 3. **Backend Abstraction**
Rendering backend is abstracted, allowing different rendering technologies.

```cpp
// OpenGL backend using existing gldraw
auto backend = std::make_unique<GlDrawBackend>();
scene_panel->SetRenderBackend(std::move(backend));

// Future: Could support Vulkan, software rendering, or mock backends
```

## Interface Architecture

### Core Components

```
VirtualScenePanel (UI Integration)
       ↓
VirtualScene (Object Management)  
       ↓  
RenderInterface (Rendering Abstraction)
       ↓
GlDrawBackend (OpenGL Implementation)
       ↓
GlSceneManager (Existing Rendering)
```

### Key Classes

1. **VirtualObject** - Base class for all interactive scene objects
   - Transform and visibility state
   - Hit testing for selection  
   - Event callbacks (OnClick, OnDrag, OnHover)
   - Backend synchronization

2. **VirtualScene** - Scene management and coordination
   - Object lifecycle (add, remove, update)
   - Selection management
   - Interaction processing
   - Event dispatching

3. **RenderInterface** - Abstract rendering interface
   - Object creation and updates
   - Rendering coordination
   - Hit testing and picking
   - Platform abstraction

4. **VirtualScenePanel** - ImGui integration
   - Input event handling
   - UI rendering coordination  
   - Scene display management

5. **EventSystem** - Event dispatching and subscription
   - Type-safe event handling
   - Filtering and batching
   - Application decoupling

## Workflow Integration

### Basic Usage Pattern

```cpp
// 1. Create scene panel (replaces SceneViewPanel)
auto scene_panel = std::make_shared<VirtualScenePanel>("3D Scene");

// 2. Set up backend (wraps existing gldraw)
scene_panel->SetRenderBackend(std::make_unique<GlDrawBackend>());

// 3. Create virtual objects with semantics
auto waypoint = CreateVirtualSphere("waypoint_001", 0.5f);
waypoint->SetPosition({-2, 0, 0});
waypoint->OnClick = [](VirtualObject* obj, ...) { /* handle click */ };

// 4. Add to scene
scene_panel->AddObject("waypoint_001", std::move(waypoint));

// 5. Subscribe to scene events
auto dispatcher = scene_panel->GetVirtualEventDispatcher();
dispatcher->Subscribe(VirtualEventType::BackgroundClicked, [...]);
```

### Event Flow

```
User Input (ImGui) 
    → VirtualScenePanel::HandleInput()
    → VirtualScene::Pick() / ProcessClick()
    → VirtualObject callbacks
    → VirtualEventDispatcher::Dispatch()  
    → Application handlers
```

### Rendering Flow

```
VirtualScenePanel::RenderInsideWindow()
    → VirtualScene::Render()
    → RenderInterface::Render()
    → GlDrawBackend → GlSceneManager
    → OpenGL rendering
```

## Implementation Strategy

### Phase 1: Interface Design ✅ (Current)
- Define all core interfaces
- Create demonstration test showing intended usage
- Validate architectural concepts
- **No implementation required yet**

### Phase 2: Minimal Implementation
- Implement basic VirtualObject and VirtualSphere
- Create GlDrawBackend wrapper around existing GlSceneManager
- Basic event system with core event types
- Simple VirtualScenePanel integration

### Phase 3: Full Implementation
- Complete virtual object hierarchy
- Advanced event system with filtering
- Multi-selection and manipulation tools
- Performance optimizations

## Benefits of This Design

### For Applications
- **Semantic Objects**: Work with waypoints, not spheres
- **Event-Driven**: Clean separation of interaction and logic
- **Type Safety**: Compile-time event type checking
- **Easy Extension**: Add new object types easily

### For QuickViz Library  
- **Clean Architecture**: Separation of concerns
- **Backwards Compatible**: Existing code continues to work
- **Platform Agnostic**: Multiple rendering backends possible
- **Professional UX**: Industry-standard interaction patterns

### For Development
- **Incremental**: Can implement piece by piece
- **Testable**: Clear interfaces enable unit testing
- **Visual**: Test with actual 3D scenes
- **Safe**: Interfaces validate feasibility before implementation

## Integration with Existing System

### SceneViewPanel Evolution
The vscene system builds on the recently completed SceneViewPanel/GlSceneManager separation:

- **SceneViewPanel** → **VirtualScenePanel** (UI integration)
- **GlSceneManager** → **RenderInterface** (rendering abstraction)  
- **OpenGlObject** → **VirtualObject** (application semantics)

### Backwards Compatibility
Existing applications can:
1. Continue using SceneViewPanel/GlSceneManager (no changes)
2. Migrate to VirtualScenePanel incrementally
3. Mix both approaches during transition

## Next Steps

1. **Validate Interfaces**: Review design with stakeholders
2. **Feasibility Check**: Ensure integration with existing gldraw system
3. **Prototype Implementation**: Start with VirtualSphere and basic GlDrawBackend
4. **Demonstration**: Build working demo with 2-3 virtual object types

## Conclusion

The vscene interface design provides a clean, professional foundation for interactive 3D visualization applications. By focusing on application-level semantics and event-driven architecture, it transforms QuickViz from a rendering library into a complete interactive 3D scene management system.

The incremental implementation strategy ensures working code at each step while building toward the full vision of professional-grade 3D interaction tools.