# Primitive Selection Extension Design

*Date: January 2025*  
*Author: Ruixiang Du*  
*Status: Design Proposal*

## Overview

This document proposes a systematic extension of QuickViz's selection system to support all geometric primitives, enabling interactive editing of graph-based visualizations, spatial networks, and hierarchical scene structures. The design maintains QuickViz's building-block philosophy while providing a complete selection framework for complex spatial data editing applications.

## Motivation

Many robotics and visualization applications require interactive editing of graph-like structures:
- **Navigation Graphs**: Waypoint networks with corridors and zones
- **Scene Graphs**: Hierarchical 3D object relationships
- **Sensor Networks**: Connected sensor nodes with coverage areas
- **Planning Graphs**: State spaces with transitions and regions
- **Infrastructure Maps**: Roads, paths, and area definitions

These applications share common visual primitives that require selection and editing capabilities.

## 1. Abstract Graph Elements to Visual Primitives

### 1.1 Element Classification

Graph-based visualizations typically consist of these abstract elements:

| Abstract Element | Visual Representation | Geometric Primitives | Interaction Purpose |
|-----------------|----------------------|---------------------|-------------------|
| **Vertices/Nodes** | 3D points in space | `Sphere`, `Box`, `Cylinder` | Position, properties, connections |
| **Edges/Links** | Connections between nodes | `Cylinder`, `LineStrip`, `Arrow` | Connectivity, flow, properties |
| **Paths/Polylines** | Multi-segment connections | `LineStrip`, `Path` | Curved routes, trajectories |
| **Regions/Areas** | 2D/3D spatial zones | `Mesh`, `Plane`, `BoundingBox` | Spatial boundaries, properties |
| **Transitions** | Special connectors | `Cylinder`, `Arrow`, `Frustum` | Level changes, state transitions |
| **Annotations** | Labels and metadata | `Text3D`, `Billboard` | Information display, selection |

### 1.2 Rendering Hierarchy

A typical scene organization for selectable graph structures:

```
Interactive Scene
├── Background Layer (non-selectable)
│   ├── Grid (reference plane)
│   ├── CoordinateFrame (orientation)
│   └── Texture (maps, imagery)
├── Region Layer (lowest selection priority)
│   ├── Mesh (polygonal areas)
│   ├── Plane (rectangular zones)
│   └── BoundingBox (3D volumes)
├── Connection Layer (medium priority)
│   ├── Cylinder (straight edges)
│   ├── LineStrip (curved paths)
│   └── Arrow (directed edges)
└── Vertex Layer (highest priority)
    ├── Sphere (point nodes)
    ├── Box (structural nodes)
    └── Text3D (labels)
```

## 2. Selection System Architecture

### 2.1 Primitive Selection Interface

Every selectable primitive must implement:

```cpp
class SelectablePrimitive {
public:
    // Core selection support
    virtual bool SupportsSelection() const = 0;
    virtual std::pair<glm::vec3, glm::vec3> GetBoundingBox() const = 0;
    
    // Visual feedback
    virtual void SetHighlighted(bool highlighted) = 0;
    virtual void SetSelectionStyle(const SelectionStyle& style) = 0;
    
    // Extended selection info
    virtual SelectionInfo GetSelectionInfo(const glm::vec3& ray_origin,
                                          const glm::vec3& ray_direction) const {
        SelectionInfo info;
        info.primitive_type = GetPrimitiveType();
        info.bounding_box = GetBoundingBox();
        return info;
    }
};
```

### 2.2 Selection Styles

Different primitive types benefit from different highlight styles:

```cpp
struct SelectionStyle {
    enum class Mode {
        kColorChange,    // Change base color
        kOutline,        // Add border/wireframe
        kGlow,          // Emission effect
        kScale,         // Size increase
        kTransparency,  // Opacity change
        kAnimation      // Pulsing/rotating
    };
    
    Mode mode = Mode::kOutline;
    glm::vec3 color = glm::vec3(1.0f, 1.0f, 0.0f);  // Yellow default
    float intensity = 1.0f;
    float line_width = 2.0f;
    bool animate = false;
};
```

## 3. Primitive-Specific Implementations

### 3.1 Cylinder (Edges/Connections)

**Use Cases**: Graph edges, corridors, pipes, structural beams

```cpp
class Cylinder : public GeometricPrimitive {
public:
    bool SupportsSelection() const override { return true; }
    
    std::pair<glm::vec3, glm::vec3> GetBoundingBox() const override {
        // Calculate AABB from cylinder endpoints and radius
        glm::vec3 min_point = glm::min(base_center_, top_center_);
        glm::vec3 max_point = glm::max(base_center_, top_center_);
        glm::vec3 radius_vec(radius_, radius_, radius_);
        return {min_point - radius_vec, max_point + radius_vec};
    }
    
    void SetHighlighted(bool highlighted) override {
        if (highlighted) {
            saved_color_ = GetColor();
            SetColor(selection_style_.color);
            if (selection_style_.mode == SelectionStyle::Mode::kOutline) {
                SetRenderMode(RenderMode::kOutline);
            }
        } else {
            SetColor(saved_color_);
            SetRenderMode(previous_mode_);
        }
    }
    
    // Cylinder-specific: get closest point on axis for precise selection
    glm::vec3 GetClosestPointOnAxis(const glm::vec3& point) const;
};
```

**Selection Behavior**:
- Ray-cylinder intersection for precise selection
- Highlight by color change or wireframe overlay
- Support for variable radius along length

### 3.2 LineStrip (Paths/Polylines)

**Use Cases**: Trajectories, curved paths, boundaries, contours

```cpp
class LineStrip : public OpenGlObject {
public:
    bool SupportsSelection() const override { return true; }
    
    std::pair<glm::vec3, glm::vec3> GetBoundingBox() const override {
        glm::vec3 min_point(FLT_MAX);
        glm::vec3 max_point(-FLT_MAX);
        for (const auto& point : points_) {
            min_point = glm::min(min_point, point);
            max_point = glm::max(max_point, point);
        }
        return {min_point, max_point};
    }
    
    void SetHighlighted(bool highlighted) override {
        if (highlighted) {
            saved_width_ = line_width_;
            saved_color_ = uniform_color_;
            SetLineWidth(line_width_ * 2.0f);  // Thicker line
            SetColor(selection_style_.color);
            if (selection_style_.mode == SelectionStyle::Mode::kGlow) {
                EnableGlowEffect(true);
            }
        } else {
            SetLineWidth(saved_width_);
            SetColor(saved_color_);
            EnableGlowEffect(false);
        }
    }
    
    // LineStrip-specific: find closest segment
    size_t GetClosestSegment(const glm::vec3& point) const;
    float GetParameterAtPoint(const glm::vec3& point) const;
};
```

**Selection Behavior**:
- Tolerance-based selection (pixel radius)
- Highlight by width increase or glow effect
- Return closest point along polyline

### 3.3 Mesh (Regions/Areas)

**Use Cases**: Zones, terrain patches, polygonal areas, surfaces

```cpp
class Mesh : public OpenGlObject {
public:
    bool SupportsSelection() const override { return true; }
    
    std::pair<glm::vec3, glm::vec3> GetBoundingBox() const override {
        // Return cached AABB from vertices
        return {aabb_min_, aabb_max_};
    }
    
    void SetHighlighted(bool highlighted) override {
        if (highlighted) {
            saved_opacity_ = GetOpacity();
            saved_color_ = GetColor();
            
            switch (selection_style_.mode) {
                case SelectionStyle::Mode::kTransparency:
                    SetOpacity(0.7f);
                    SetColor(selection_style_.color);
                    break;
                case SelectionStyle::Mode::kOutline:
                    EnableEdgeRendering(true);
                    SetEdgeColor(selection_style_.color);
                    SetEdgeWidth(selection_style_.line_width);
                    break;
            }
        } else {
            SetOpacity(saved_opacity_);
            SetColor(saved_color_);
            EnableEdgeRendering(false);
        }
    }
    
    // Mesh-specific: precise triangle intersection
    bool RayIntersectsTriangle(const glm::vec3& ray_origin,
                               const glm::vec3& ray_direction,
                               size_t triangle_index,
                               float& distance) const;
};
```

**Selection Behavior**:
- Ray-triangle intersection for precise selection
- Highlight by transparency or edge rendering
- Support for per-face or per-vertex selection

### 3.4 Additional Primitives

**Text3D** (Labels):
- Bounding box from text dimensions
- Highlight by background color or outline
- Click-through option for labels

**Arrow** (Directed Edges):
- Bounding box from shaft and head
- Highlight entire arrow or just head
- Direction visualization enhancement

**Plane** (2D Regions):
- Simple AABB from corners
- Highlight by border or fill
- Grid overlay option

**BoundingBox** (3D Volumes):
- Already is its own bounding box
- Highlight by edge color/width
- Transparent fill option

## 4. Selection Modes and Filters

### 4.1 Selection Mode System

```cpp
class SelectionModeManager {
public:
    enum class Mode {
        kAll,           // Select any primitive
        kVertices,      // Only point-like objects
        kEdges,         // Only connection objects
        kRegions,       // Only area objects
        kAnnotations,   // Only text/labels
        kByType,        // Custom type filter
        kByLayer        // Layer-based filter
    };
    
    void SetMode(Mode mode);
    void SetCustomFilter(std::function<bool(const SelectionResult&)> filter);
    
    bool IsSelectable(const SelectionResult& result) const;
};
```

### 4.2 Multi-Selection Strategies

```cpp
class MultiSelectionStrategy {
public:
    enum class Method {
        kReplace,       // New selection replaces old
        kAdd,           // Add to selection (Ctrl+Click)
        kToggle,        // Toggle selection state (Ctrl+Alt+Click)
        kRemove,        // Remove from selection (Alt+Click)
        kIntersect,     // Keep only intersection
        kConnected      // Select connected components
    };
    
    void UpdateSelection(SelectionResult& current,
                        const SelectionResult& new_selection,
                        Method method);
};
```

## 5. Implementation Roadmap

### Phase 1: Core Infrastructure (Week 1)
- [ ] Define `SelectablePrimitive` interface
- [ ] Implement `SelectionStyle` system
- [ ] Create `SelectionModeManager`
- [ ] Add selection support base class

### Phase 2: Primary Primitives (Week 1-2)
- [ ] Cylinder - full selection implementation
- [ ] LineStrip - polyline selection with tolerance
- [ ] Mesh - triangle-accurate selection
- [ ] Sphere - update existing implementation

### Phase 3: Secondary Primitives (Week 2-3)
- [ ] Text3D - clickable labels
- [ ] Arrow - directional indicators
- [ ] Plane - 2D regions
- [ ] BoundingBox - volume selection
- [ ] Path - specialized trajectory selection

### Phase 4: Advanced Features (Week 3-4)
- [ ] Multi-selection with different strategies
- [ ] Selection groups and hierarchies
- [ ] Proximity-based selection
- [ ] Lasso and box selection tools
- [ ] Selection history (undo/redo)

## 6. Usage Examples

### 6.1 Graph Visualization Editor

```cpp
class GraphEditor : public quickviz::Application {
public:
    void CreateGraph() {
        // Create vertices as spheres
        for (const auto& vertex : graph_.vertices) {
            auto sphere = std::make_unique<Sphere>(vertex.position, 0.5f);
            sphere->SetColor(GetVertexColor(vertex.type));
            scene_->AddObject(vertex.id, std::move(sphere));
        }
        
        // Create edges as cylinders or linestrips
        for (const auto& edge : graph_.edges) {
            if (edge.is_straight) {
                auto cylinder = std::make_unique<Cylinder>(
                    GetVertexPosition(edge.from),
                    GetVertexPosition(edge.to),
                    edge.width / 2.0f
                );
                scene_->AddObject(edge.id, std::move(cylinder));
            } else {
                auto linestrip = std::make_unique<LineStrip>();
                linestrip->SetPoints(edge.waypoints);
                linestrip->SetLineWidth(edge.width);
                scene_->AddObject(edge.id, std::move(linestrip));
            }
        }
        
        // Setup selection handling
        scene_->SetSelectionCallback([this](const SelectionResult& result) {
            OnElementSelected(result);
        });
    }
    
private:
    void OnElementSelected(const SelectionResult& result) {
        // Update property panel
        // Enable appropriate editing tools
        // Highlight connected elements
    }
};
```

### 6.2 Spatial Zone Editor

```cpp
class ZoneEditor : public quickviz::Application {
public:
    void CreateZone(const Polygon& polygon) {
        auto mesh = std::make_unique<Mesh>();
        mesh->LoadFromPolygon(polygon.vertices);
        mesh->SetColor(GetZoneColor(polygon.type));
        mesh->SetOpacity(0.3f);
        
        // Enable selection with custom style
        SelectionStyle style;
        style.mode = SelectionStyle::Mode::kOutline;
        style.color = glm::vec3(1, 1, 0);
        style.line_width = 3.0f;
        mesh->SetSelectionStyle(style);
        
        scene_->AddObject(polygon.id, std::move(mesh));
    }
    
    void EnableZoneEditMode() {
        // Set selection filter to only select meshes
        selection_manager_->SetMode(SelectionModeManager::Mode::kRegions);
        
        // Enable vertex editing for selected mesh
        selection_manager_->SetSelectionCallback([this](const SelectionResult& result) {
            if (auto mesh = GetSelectedMesh(result)) {
                StartVertexEditing(mesh);
            }
        });
    }
};
```

## 7. Performance Considerations

### 7.1 Bounding Box Caching
- Cache AABB for static objects
- Update only on transformation changes
- Use spatial indices for large scenes

### 7.2 Selection Optimization
- Hierarchical bounding volumes for complex meshes
- LOD-based selection for distant objects
- Frustum culling before selection testing

### 7.3 Visual Feedback Performance
- Minimize state changes for highlighting
- Use uniform buffers for selection colors
- Batch similar highlight styles

## 8. Testing Strategy

### 8.1 Unit Tests
- Bounding box calculation for each primitive
- Ray-primitive intersection accuracy
- Selection filter correctness

### 8.2 Integration Tests
- Multi-selection scenarios
- Selection mode switching
- Performance with large scenes

### 8.3 Visual Tests
- Highlight style rendering
- Selection feedback responsiveness
- Edge cases (overlapping objects)

## 9. Benefits

1. **Completeness**: All QuickViz primitives become selectable
2. **Consistency**: Unified selection interface across primitive types
3. **Flexibility**: Customizable selection styles and modes
4. **Performance**: Optimized per-primitive selection methods
5. **Reusability**: Generic design works for any graph-like visualization
6. **Extensibility**: Easy to add new primitive types

## 10. Conclusion

This design provides a comprehensive framework for extending selection capabilities to all QuickViz primitives. By maintaining generic, reusable components while supporting specialized selection behaviors, the system enables sophisticated interactive editing applications while preserving the library's building-block philosophy.

The implementation prioritizes the most commonly needed primitives (Cylinder, LineStrip, Mesh) while providing clear extension points for future primitive types. The selection system remains agnostic to specific application domains, making it suitable for navigation graphs, scene editing, sensor networks, or any other graph-based visualization needs.