# QuickViz Architecture Design

*Last Updated: January 22, 2025*

## Overview

QuickViz follows a layered architecture with clear separation between **rendering** and **data visualization**. This design enables both high-performance rendering and flexible data integration.

## Core Architectural Principles

1. **Separation of Concerns**: Rendering logic separated from data conversion logic
2. **Dependency Direction**: Higher-level modules depend on lower-level ones, never the reverse
3. **Single Responsibility**: Each module has one well-defined purpose
4. **Extensibility**: Easy to add new data types and visualization methods
5. **Performance**: Low-level rendering optimized independently of high-level APIs

## Module Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Applications                             │
│                   (External Processing)                        │
└─────────────────────────────┬───────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    visualization Module                        │  ← High-level data mapping
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │ Data Contracts  │  │   Renderables   │  │    Helpers      │ │
│  │ SelectionData   │  │SelectionRendr.. │  │ Convenience     │ │
│  │ SurfaceData     │  │SurfaceRendr...  │  │ Functions       │ │
│  │ TrajectoryData  │  │TrajectoryRendr..│  │ Factory Methods │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
└─────────────────────────────┬───────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                       gldraw Module                            │  ← Pure rendering engine
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │   Renderables   │  │ Scene Manager   │  │   PCL Bridge    │ │
│  │ PointCloud      │  │ GlSceneManager  │  │ File Loading    │ │
│  │ Mesh            │  │ Camera          │  │ Format Support  │ │
│  │ Grid, Triangle  │  │ Layers          │  │ Conversions     │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
└─────────────────────────────┬───────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Lower-level Modules                         │
│               imview, core, OpenGL, PCL, etc.                  │
└─────────────────────────────────────────────────────────────────┘
```

## Module Responsibilities

### **gldraw Module** (Low-level rendering)

**Purpose**: Efficient OpenGL rendering of primitive geometric objects

**Responsibilities**:
- OpenGL resource management (VAOs, VBOs, shaders)
- Primitive renderable objects (PointCloud, Mesh, Grid, etc.)
- Scene management and camera controls
- Layer composition and rendering pipeline
- PCL file loading and format conversion

**Key Classes**:
- `OpenGlObject` - Base interface for all renderable objects
- `GlSceneManager` - Scene composition and rendering coordination
- `PointCloud` - Point-based rendering with layer support
- `Mesh` - Triangle mesh rendering with materials
- `Camera`, `CameraController` - 3D navigation

**API Style**: Low-level, performance-focused
```cpp
auto mesh = std::make_unique<Mesh>();
mesh->SetVertices(vertices);
mesh->SetIndices(indices);
scene.AddOpenGLObject("my_mesh", std::move(mesh));
```

### **visualization Module** (High-level data mapping)

**Purpose**: Convert domain-specific data into renderable objects

**Responsibilities**:
- Data contract definitions (SelectionData, SurfaceData, etc.)
- Data validation and error handling
- Conversion from external data to gldraw renderables
- Convenience APIs for common visualization tasks
- Mock data generation for testing

**Key Classes**:
- `SelectionRenderable` - Converts SelectionData to PointCloud highlights
- `SurfaceRenderable` - Converts SurfaceData to Mesh objects
- `TrajectoryRenderable` - Converts path data to line/curve rendering
- Helper factories and convenience functions

**API Style**: High-level, domain-focused
```cpp
auto selection = SelectionRenderable::FromData(selection_data, target_cloud);
auto surface = SurfaceRenderable::FromData(surface_data);
scene.AddOpenGLObject("selection", std::move(selection));
```

## Data Flow

### 1. External Processing → Data Contracts
```cpp
// External algorithm produces domain data
std::vector<size_t> selected_indices = my_algorithm.ProcessPointCloud();

// Convert to visualization data contract
visualization::SelectionData selection;
selection.point_indices = selected_indices;
selection.highlight_color = glm::vec3(1.0f, 0.0f, 0.0f);
selection.selection_name = "segmentation_result";
```

### 2. Data Contracts → Renderable Objects
```cpp
// Visualization module converts data to renderables
auto selection_renderable = visualization::SelectionRenderable::FromData(
    selection, target_point_cloud);
auto surface_renderable = visualization::SurfaceRenderable::FromData(surface);
```

### 3. Renderable Objects → Scene Rendering
```cpp
// gldraw module handles pure rendering
GlSceneManager scene;
scene.AddOpenGLObject("selection", std::move(selection_renderable));
scene.AddOpenGLObject("surface", std::move(surface_renderable));
scene.Draw(); // Efficient OpenGL rendering
```

## Benefits of This Architecture

### **Separation of Concerns**
- **gldraw**: Focused solely on efficient rendering
- **visualization**: Focused solely on data conversion
- **Applications**: Focused solely on domain logic

### **Independent Evolution**
- Rendering optimizations don't affect data APIs
- New data types don't require rendering changes
- Performance improvements isolated to appropriate layers

### **Better Testing**
- Unit test rendering without data conversion complexity
- Unit test data conversion without OpenGL setup
- Integration tests at appropriate boundaries

### **Easier Maintenance**
- Changes to data formats contained in visualization module
- Rendering bugs isolated to gldraw module
- Clear ownership of responsibilities

### **Third-party Integration**
- Other teams can create custom visualization modules
- gldraw becomes reusable rendering engine
- Clean interfaces for external contributions

## Usage Patterns

### **Simple Visualization**
```cpp
#include "visualization/helpers.hpp"

// One-line convenience functions
auto selection = visualization::CreateSelection(selection_data, cloud);
auto surface = visualization::CreateSurface(surface_data);

scene.AddOpenGLObject("sel", std::move(selection));
scene.AddOpenGLObject("surf", std::move(surface));
```

### **Custom Rendering**
```cpp
#include "gldraw/renderable/mesh.hpp"

// Direct gldraw usage for custom geometry
auto custom_mesh = std::make_unique<Mesh>();
custom_mesh->SetVertices(my_vertices);
custom_mesh->SetColor(my_color);
scene.AddOpenGLObject("custom", std::move(custom_mesh));
```

### **Mixed Usage**
```cpp
// Combine high-level visualization with low-level rendering
auto processed_selection = visualization::SelectionRenderable::FromData(data, cloud);
auto custom_grid = std::make_unique<Grid>(spacing, size, color);

scene.AddOpenGLObject("selection", std::move(processed_selection));
scene.AddOpenGLObject("grid", std::move(custom_grid));
```

## Migration Strategy

### **Phase 1**: Create visualization Module
- New module structure and build system
- Move data contracts from gldraw to visualization
- Implement core renderable classes

### **Phase 2**: Clean Interface  
- Remove scene-level methods from GlSceneManager
- Create convenience helpers in visualization
- Update documentation and examples

### **Phase 3**: Full Migration
- Update all tests to use new architecture
- Deprecate old APIs in gldraw
- Clean up legacy visualization code

## Directory Structure

```
src/
├── visualization/                    # High-level data mapping
│   ├── include/visualization/
│   │   ├── contracts/               # Data format definitions
│   │   │   ├── selection_data.hpp
│   │   │   ├── surface_data.hpp
│   │   │   └── trajectory_data.hpp
│   │   ├── renderables/             # Data-to-renderable converters
│   │   │   ├── selection_renderable.hpp
│   │   │   ├── surface_renderable.hpp
│   │   │   └── trajectory_renderable.hpp
│   │   ├── helpers/                 # Convenience functions
│   │   │   └── visualization_helpers.hpp
│   │   └── testing/                 # Mock data and utilities
│   │       └── mock_data_generator.hpp
│   ├── src/                         # Implementation files
│   └── test/                        # Visualization-specific tests
│
├── gldraw/                          # Pure rendering engine
│   ├── include/gldraw/
│   │   ├── renderable/              # OpenGL primitives
│   │   │   ├── point_cloud.hpp
│   │   │   ├── mesh.hpp
│   │   │   └── grid.hpp
│   │   ├── gl_scene_manager.hpp     # Scene management
│   │   └── pcl_bridge/              # File loading
│   ├── src/                         # Implementation files
│   └── test/                        # Rendering tests
│
└── [other modules...]               # imview, core, etc.
```

This architecture provides a solid foundation for scalable, maintainable visualization software with clear separation of concerns and excellent extensibility.