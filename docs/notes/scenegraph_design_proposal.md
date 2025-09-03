# SceneGraph State Management System Design Proposal

**Date**: September 3, 2025  
**Status**: Design Phase  
**Author**: Claude Code Assistant

## Executive Summary

This proposal outlines the design and implementation of a comprehensive state management system for QuickViz, addressing the dual needs of real-time robotics visualization and interactive point cloud editing. The solution introduces a lightweight, modal `scenegraph` module that integrates seamlessly with existing QuickViz architecture while maintaining high performance and industrial-grade features.

## Problem Statement

QuickViz currently lacks unified state management, leading to several limitations:

1. **No undo/redo system** for interactive editing operations
2. **Inconsistent state handling** between real-time and interactive modes
3. **No scene persistence** or serialization capabilities
4. **Fragmented selection management** across different components
5. **Performance overhead concerns** for real-time applications

## Design Goals

### Primary Objectives
- **Dual-mode operation**: Support both real-time visualization (minimal overhead) and interactive editing (full features)
- **Industrial practices**: Follow Unity/Unreal/Blender patterns for professional-grade tools
- **Lightweight integration**: Minimal changes to existing codebase
- **Optional adoption**: Backward compatibility with existing applications
- **Performance flexibility**: Zero overhead for real-time use cases

### Secondary Objectives
- Scene serialization and persistence
- Multi-application state synchronization
- Plugin architecture support
- Network streaming capabilities

## Use Case Analysis

### Real-Time Robotics Visualization
**Requirements**: 60+ FPS, minimal latency, streaming sensor data
- **State needs**: Lightweight updates, no history tracking
- **Examples**: SLAM visualization, sensor streaming, robot teleoperation

### Interactive Point Cloud Editing  
**Requirements**: Selection, manipulation, undo/redo, persistence
- **State needs**: Full state tracking, command history, serialization
- **Examples**: Point cloud annotation, cleanup, manual registration

### Scientific Data Analysis
**Requirements**: Large datasets, multiple synchronized views
- **State needs**: Data management, view coordination
- **Examples**: Mesh analysis, trajectory visualization, comparative studies

### Offline Rendering & Documentation
**Requirements**: Batch processing, high quality output, reproducible results
- **State needs**: Complete scene serialization, render settings persistence
- **Examples**: Paper figures, documentation, automated reporting

## Architectural Design

### Core Philosophy: Modal State Management

The system operates in three distinct modes to optimize for different use cases:

```cpp
enum class StateMode {
    kDirect,     // Real-time: bypass state tracking (zero overhead)
    kImmediate,  // Standard: track state, no command history  
    kRecorded    // Editing: full undo/redo with command recording
};
```

### System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     Application                         │
├─────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌─────────────────┐              │
│  │ Real-time Mode  │  │ Interactive Mode │              │  
│  │ (Direct)        │  │ (Recorded)       │              │
│  └─────────────────┘  └─────────────────┘              │
├─────────────────────────────────────────────────────────┤
│               scenegraph (State Layer)                  │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────────┐   │
│  │ SceneState  │ │ CommandStack│ │ SceneSerializer │   │ 
│  └─────────────┘ └─────────────┘ └─────────────────┘   │
├─────────────────────────────────────────────────────────┤
│                gldraw (Rendering Layer)                 │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────────┐   │
│  │SceneManager │ │ OpenGlObject│ │ Tools/Selection │   │
│  └─────────────┘ └─────────────┘ └─────────────────┘   │
├─────────────────────────────────────────────────────────┤
│               imview (UI Layer)                         │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────────┐   │
│  │   Viewer    │ │ SceneObject │ │     Panel       │   │
│  └─────────────┘ └─────────────┘ └─────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

## Module Design Specifications

### 1. SceneGraph Module (New)

#### SceneState - Core State Container

```cpp
namespace quickviz {

struct ObjectState {
    glm::mat4 transform = glm::mat4(1.0f);
    bool visible = true;
    bool selected = false;
    nlohmann::json metadata;  // Extensible properties
};

class SceneState {
public:
    // === Mode Management ===
    void SetMode(StateMode mode);
    StateMode GetMode() const;
    bool IsRealTimeMode() const;
    
    // === State Operations ===
    void SetObjectState(const std::string& id, const ObjectState& state);
    ObjectState GetObjectState(const std::string& id) const;
    
    // === Selection Management ===
    void Select(const std::string& id, bool add_to_selection = false);
    void ClearSelection();
    std::vector<std::string> GetSelection() const;
    
    // === Command System (Recorded mode only) ===
    void ExecuteCommand(std::unique_ptr<Command> cmd);
    void Undo();
    void Redo();
    
    // === Performance Optimization ===
    void BeginBatch();  // Suspend notifications
    void EndBatch();    // Send batched changes
    
    // === Observer Pattern ===
    using ChangeCallback = std::function<void(const std::string&)>;
    uint32_t Subscribe(ChangeCallback callback);
    void Unsubscribe(uint32_t token);
    
    // === Serialization ===
    nlohmann::json Serialize() const;
    void Deserialize(const nlohmann::json& data);
};

}
```

#### Command Pattern Implementation

```cpp
// Base command interface (in core module)
class Command {
public:
    virtual ~Command() = default;
    virtual void Execute() = 0;
    virtual void Undo() = 0;
    virtual size_t GetMemorySize() const = 0;
    virtual std::string GetDescription() const = 0;
};

// Scene-specific commands
class TransformCommand : public Command {
public:
    TransformCommand(SceneState* state, const std::string& id,
                    const glm::mat4& old_transform,
                    const glm::mat4& new_transform);
    
    void Execute() override;
    void Undo() override;
    std::string GetDescription() const override;
};

class DeletePointsCommand : public Command {
public:
    DeletePointsCommand(PointCloud* cloud, 
                       const std::vector<size_t>& point_indices);
    
    void Execute() override;
    void Undo() override;  // Restore deleted points
    size_t GetMemorySize() const override;
};
```

### 2. GLDraw Module Enhancements

#### OpenGlObject Base Class Updates

```cpp
// Add transform support to all OpenGL objects
class OpenGlObject {
    // ... existing methods ...
    
    // NEW: Transform interface
    virtual void SetTransform(const glm::mat4& transform);
    virtual glm::mat4 GetTransform() const;
    
    // NEW: Visibility interface  
    virtual void SetVisible(bool visible);
    virtual bool IsVisible() const;
    
protected:
    glm::mat4 transform_ = glm::mat4(1.0f);
    bool visible_ = true;
};
```

#### SceneManager Integration

```cpp
class SceneManager {
    // ... existing methods ...
    
    // NEW: Optional state management
    void SetSceneState(std::shared_ptr<SceneState> state);
    std::shared_ptr<SceneState> GetSceneState() const;
    
    // NEW: Command execution
    void ExecuteCommand(std::unique_ptr<Command> cmd);
    void Undo();
    void Redo();
    
    // NEW: Direct update API (for real-time mode)
    void UpdateObjectDirect(const std::string& name, 
                           const glm::mat4& transform);
    
private:
    std::shared_ptr<SceneState> scene_state_;  // Optional
    
    void OnStateChanged(const std::string& object_id);
};
```

### 3. Core Module Additions

#### Command Stack Management

```cpp
namespace quickviz {

class CommandStack {
public:
    void Execute(std::unique_ptr<Command> command);
    void Undo();
    void Redo();
    
    bool CanUndo() const;
    bool CanRedo() const;
    
    void Clear();
    void SetMaxSize(size_t max_commands);
    
    // Memory management
    size_t GetMemoryUsage() const;
    void CompactHistory();  // Remove redundant commands
    
private:
    std::deque<std::unique_ptr<Command>> undo_stack_;
    std::deque<std::unique_ptr<Command>> redo_stack_;
    size_t max_size_ = 100;
    size_t memory_limit_ = 1024 * 1024 * 1024;  // 1GB default
};

}
```

## Integration Examples

### Real-Time Sensor Streaming

```cpp
// Zero overhead mode for real-time applications
class RobotVizualizer {
public:
    void InitializeRealTimeMode() {
        scene_manager_ = std::make_unique<SceneManager>("realtime");
        // Don't set SceneState - direct updates only
    }
    
    void UpdateSensorData(const SensorFrame& frame) {
        // Direct updates bypass state management entirely
        scene_manager_->UpdateObjectDirect("lidar", frame.lidar_transform);
        scene_manager_->UpdateObjectDirect("camera", frame.camera_transform);
        // No memory allocation, no state tracking, maximum performance
    }
};
```

### Interactive Point Cloud Editor

```cpp
class PointCloudEditor {
public:
    void InitializeEditingMode() {
        scene_manager_ = std::make_unique<SceneManager>("editor");
        scene_state_ = std::make_shared<SceneState>();
        scene_state_->SetMode(StateMode::kRecorded);  // Full features
        scene_manager_->SetSceneState(scene_state_);
    }
    
    void DeleteSelectedPoints() {
        auto selected_points = GetSelectedPointIndices();
        auto cmd = std::make_unique<DeletePointsCommand>(
            point_cloud_, selected_points);
        scene_manager_->ExecuteCommand(std::move(cmd));
        // Automatic undo/redo support
    }
    
    void UndoLastAction() {
        scene_manager_->Undo();
    }
    
    void SaveProject(const std::string& path) {
        SceneIO::SaveToFile(*scene_state_, path);
    }
};
```

### Hybrid Application (Mode Switching)

```cpp
class AdaptiveRoboticsApp {
    void StartLiveVisualization() {
        scene_state_->SetMode(StateMode::kDirect);
        selection_tools_->SetEnabled(false);  // Disable expensive features
        
        // Batch updates for performance
        scene_state_->BeginBatch();
        for (const auto& update : sensor_updates) {
            scene_manager_->UpdateObjectDirect(update.object_id, update.transform);
        }
        scene_state_->EndBatch();
    }
    
    void StartInteractiveEditing() {
        scene_state_->SetMode(StateMode::kRecorded);
        selection_tools_->SetEnabled(true);
        tool_manager_->ActivateTool("point_selection");
    }
};
```

## File Organization

```
src/
├── core/                    # Enhanced
│   └── include/core/
│       └── command/         # NEW
│           ├── command.hpp
│           └── command_stack.hpp
├── scenegraph/              # NEW MODULE  
│   ├── include/scenegraph/
│   │   ├── scene_state.hpp
│   │   ├── object_state.hpp
│   │   ├── scene_commands.hpp
│   │   ├── scene_io.hpp
│   │   └── scene_factory.hpp
│   ├── src/
│   │   ├── scene_state.cpp
│   │   ├── commands/
│   │   │   ├── transform_command.cpp
│   │   │   ├── delete_points_command.cpp
│   │   │   └── create_object_command.cpp
│   │   ├── scene_io.cpp
│   │   └── scene_factory.cpp
│   ├── test/
│   │   ├── test_scene_state.cpp
│   │   ├── test_commands.cpp
│   │   └── test_serialization.cpp
│   └── CMakeLists.txt
├── gldraw/                  # Enhanced
│   ├── interface/
│   │   └── opengl_object.hpp    # Add transform support
│   ├── scene_manager.hpp        # Add state integration
│   └── renderers/               # NEW
│       └── stateful_renderer.hpp
└── imview/                  # No changes required
```

## Performance Considerations

### Memory Management
- **Lazy allocation**: Command stack only created in Recorded mode
- **Memory limits**: Configurable bounds on undo/redo history
- **Smart compression**: Remove redundant transform commands
- **Batch operations**: Group updates to minimize notifications

### Real-Time Optimization
- **Direct mode**: Zero state tracking overhead
- **Batch updates**: Group notifications for efficiency
- **Selective features**: Disable expensive features in real-time mode
- **Memory pools**: Reuse command objects to avoid allocation

### Large Dataset Handling
- **Incremental serialization**: Save only changed data
- **Compression**: Use binary formats for large point clouds
- **Streaming**: Load/save data in chunks
- **Background processing**: Non-blocking I/O operations

## Implementation Phases

### Phase 1: Foundation (Week 1)
- [ ] Create command interface in core module
- [ ] Implement basic CommandStack
- [ ] Add transform support to OpenGlObject
- [ ] Unit tests for command pattern

### Phase 2: SceneState Core (Week 2)  
- [ ] Implement SceneState with three modes
- [ ] Add observer pattern for change notifications
- [ ] Create basic commands (Transform, Visibility)
- [ ] Integration tests with SceneManager

### Phase 3: Advanced Commands (Week 3)
- [ ] Implement DeletePointsCommand with undo
- [ ] Add point cloud editing operations
- [ ] Create CompoundCommand for complex operations
- [ ] Memory management and limits

### Phase 4: Serialization (Week 4)
- [ ] JSON schema design for scene format
- [ ] Implement SceneIO with versioning
- [ ] Binary blob support for large data
- [ ] Round-trip serialization tests

### Phase 5: Tool Integration (Week 5)
- [ ] Update PointSelectionTool to use commands
- [ ] Add state-aware transform tools
- [ ] Create SceneFactory for different modes
- [ ] Performance benchmarks and optimization

## Testing Strategy

### Unit Tests
- Command execution and undo/redo correctness
- State management in all three modes
- Serialization round-trip integrity
- Memory usage and limits enforcement

### Integration Tests  
- SceneManager + SceneState interaction
- Tool integration with command system
- Multi-mode switching scenarios
- Large dataset handling

### Performance Tests
- Real-time mode overhead measurement
- Memory usage under different workloads
- Serialization/deserialization benchmarks
- Batch operation efficiency

### Stress Tests
- Large undo/redo histories
- Memory limit boundary conditions
- Concurrent access scenarios
- Network serialization/deserialization

## Migration Strategy

### Backward Compatibility
- All existing QuickViz applications continue to work unchanged
- SceneState integration is completely optional
- Direct rendering path preserved for maximum performance
- Gradual tool migration without breaking existing workflows

### Incremental Adoption
1. **Phase 1**: Applications can use new command pattern without state
2. **Phase 2**: Optional SceneState integration for advanced features  
3. **Phase 3**: Full migration to command-based operations
4. **Phase 4**: Legacy direct-modification APIs can be deprecated

## Future Extensions

### Advanced Features
- **Animation system**: Keyframe interpolation using state history
- **Collaborative editing**: Network state synchronization
- **Plugin architecture**: External tools using command API
- **Macro recording**: Record and replay user action sequences

### Performance Optimizations
- **GPU state management**: Parallel state updates on GPU
- **Incremental updates**: Delta-based change propagation
- **Spatial indexing**: Efficient queries on large scenes
- **Level-of-detail**: Automatic quality adjustment for performance

## Conclusion

This design provides QuickViz with professional-grade state management while maintaining its lightweight, high-performance characteristics. The modal architecture ensures that real-time applications get zero overhead, while interactive applications get full-featured editing capabilities.

The implementation follows industrial best practices and integrates seamlessly with QuickViz's existing architecture. The gradual migration path ensures that existing applications continue to work while new capabilities are introduced incrementally.

This foundation enables QuickViz to support advanced use cases like collaborative robotics visualization, professional point cloud editing tools, and complex multi-view scientific applications, while remaining true to its core philosophy of being a lightweight, composable visualization toolkit.