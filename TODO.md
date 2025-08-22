# QuickViz gldraw Development Roadmap

*Last Updated: January 22, 2025 - Architecture Redesign*

## Project Vision

Create a **layered visualization architecture** with clear separation between rendering and data conversion:
- **gldraw module**: Pure OpenGL rendering engine for geometric primitives
- **visualization module**: High-level data mapping from external formats to renderables
- **Clean interfaces**: Domain-specific data contracts with efficient rendering backend

This design enables high-performance rendering while providing flexible, extensible APIs for external processing integration.

## Target Architecture

```
src/
├── visualization/                   # 🆕 High-level data mapping module
│   ├── contracts/                   # Data format definitions
│   │   ├── selection_data.hpp       # Point selection specifications
│   │   ├── surface_data.hpp         # Mesh/surface specifications  
│   │   └── trajectory_data.hpp      # Path/motion specifications
│   ├── renderables/                 # Data-to-renderable converters
│   │   ├── selection_renderable.hpp # SelectionData → PointCloud highlights
│   │   ├── surface_renderable.hpp   # SurfaceData → Mesh objects
│   │   └── trajectory_renderable.hpp# TrajectoryData → Line/curve objects
│   ├── helpers/                     # Convenience factory functions
│   └── testing/                     # Mock data generators
│
└── gldraw/                          # ✅ Pure OpenGL rendering engine
    ├── renderable/                  # Core geometric primitives
    │   ├── point_cloud.hpp          # ✅ Point-based rendering with layers
    │   ├── mesh.hpp                 # ✅ Triangle mesh with materials
    │   ├── grid.hpp, triangle.hpp   # ✅ Basic primitives
    │   └── coordinate_frame.hpp     # ✅ 3D reference frames
    ├── gl_scene_manager.hpp         # ✅ Scene composition and rendering
    ├── pcl_bridge/                  # ✅ File loading and format support
    └── test/                        # ✅ Rendering-focused tests
```

## Development Status Overview

### ✅ gldraw Module (Pure Rendering Engine)
- **Point Cloud Rendering**: RGB, intensity, height field visualization with layer support
- **Mesh Rendering**: Triangle mesh visualization with transparency, wireframe, and materials
- **Scene Management**: `GlSceneManager` for efficient object composition and rendering
- **Basic Primitives**: Grid, Triangle, CoordinateFrame for reference geometry
- **Camera System**: 3D navigation with mouse/keyboard controls
- **PCL Integration**: File loading, format conversion, and data import
- **Performance**: Optimized OpenGL pipeline with proper resource management

### 🔄 Architecture Transition (In Progress)
- **Current**: gldraw contains both rendering and data conversion (Phase 4 implementation)
- **Target**: Clean separation between gldraw (rendering) and visualization (data mapping)
- **Migration**: Moving data contracts and conversion logic to new visualization module

### 📋 visualization Module (High-level Data Mapping)
- **Data Contracts**: Standardized formats for external processing results
- **Renderable Converters**: Transform domain data into gldraw objects
- **Convenience APIs**: Simple factory methods for common visualization tasks
- **Testing Support**: Mock data generators and validation utilities

---

## Development Phases

### ✅ Previous Development (Phases 1-4) - Foundation Complete
- **Core Rendering**: Point clouds, meshes, layers, camera controls
- **PCL Integration**: File loading, format support, data conversion
- **Data Contracts**: External API definitions (SelectionData, SurfaceData)
- **Initial Visualization**: Proof-of-concept for external data integration

*Note: Previous phases established the foundation but mixed rendering and data conversion concerns in a single module. The architecture redesign separates these for better maintainability.*

### 🔄 Phase 5: Architecture Redesign (Current)
**Goal**: Create clean separation between rendering and data mapping

**Task List**:
- [ ] **New Module Structure**: Create independent visualization module
- [ ] **Move Data Contracts**: Migrate contracts from gldraw to visualization
- [ ] **Renderable Classes**: Implement SelectionRenderable, SurfaceRenderable
- [ ] **Clean gldraw**: Remove data conversion logic, focus on pure rendering
- [ ] **Update Tests**: Demonstrate new architecture with working examples
- [ ] **Migration Guide**: Document transition from old to new APIs

**Target Architecture**:
```cpp
// New clean separation
#include "visualization/renderables/selection_renderable.hpp"
#include "gldraw/gl_scene_manager.hpp"

// Data → Renderable (visualization module)
auto selection = visualization::SelectionRenderable::FromData(selection_data, cloud);

// Renderable → Scene (gldraw module)  
scene.AddOpenGLObject("selection", std::move(selection));
```

### Phase 6: Enhanced Data Contracts (Future)
**Goal**: Support for complex processing results in visualization module

**Planned Contracts**:
- [ ] `ClusterData` - Point cloud clustering results with boundary visualization
- [ ] `TrajectoryData` - Robot paths and motion planning with temporal encoding
- [ ] `AnnotationData` - Labels, measurements, text overlays in 3D space
- [ ] `StatisticsData` - Per-point analysis results with color mapping

**Corresponding Renderables**:
- [ ] `ClusterRenderable` - Multi-cluster visualization with boundaries
- [ ] `TrajectoryRenderable` - Path visualization with velocity/time encoding
- [ ] `AnnotationRenderable` - 3D text and measurement overlays
- [ ] `StatisticsRenderable` - Data-driven point cloud coloring

### Phase 7: Performance Optimization (Future)
**Goal**: Handle large datasets efficiently

**gldraw Optimizations**:
- [ ] Level-of-detail (LOD) system for 1M+ point clouds
- [ ] GPU-optimized rendering pipeline improvements
- [ ] Memory-efficient buffer management
- [ ] Instanced rendering for repeated geometry

**visualization Optimizations**:
- [ ] Streaming data conversion for large datasets
- [ ] Cached renderable object pools
- [ ] Progressive loading for real-time algorithms
- [ ] Efficient data validation and error handling

---

## Architecture Principles

### Separation of Concerns
- **gldraw**: Pure visualization and rendering
- **External Libraries**: Data processing and algorithms  
- **Bridge Pattern**: Clean data contracts for integration

### Design Goals
1. **Zero Coupling**: External processing independent of rendering internals
2. **High Performance**: Optimized OpenGL rendering pipeline
3. **Easy Integration**: Simple APIs for common visualization tasks
4. **Backward Compatible**: Existing code continues to work
5. **Extensible**: Easy to add new data types and visualizers

### Data Flow Pattern
```
External Processing → Data Contracts → Visualizers → Scene Manager → OpenGL
```

---

## Testing Strategy

### Current Test Coverage
- ✅ **Unit Tests**: Selection algorithms, layer management, PCL conversions
- ✅ **Integration Tests**: End-to-end workflows with point cloud loading
- ✅ **Contract Tests**: Data contract validation and mock generation
- ✅ **Memory Tests**: Leak detection and resource management

### Test Files
```
src/gldraw/test/
├── test_visualization_contracts.cpp     ✅ Data contract validation
├── test_selection_visualizer.cpp       ✅ External selection visualization
├── test_scene_visualization.cpp        ✅ Scene-level integration (Phase 4)
├── test_pcd_with_selection.cpp         ✅ External selection demo with PCD files
├── test_layer_system.cpp               ✅ Multi-layer rendering
├── test_pcl_bridge.cpp                 ✅ PCL integration
└── performance/                         📋 Large dataset benchmarks
```

---

## Example Usage Patterns

### External Selection Visualization
```cpp
// 1. Load point cloud
auto factory_result = pcl_bridge::RendererFactory::Load("cloud.pcd");

// 2. Create visualization
GlSceneManager scene;
auto cloud = std::make_unique<PointCloud>();
cloud->SetPoints(factory_result.points_3d, factory_result.colors_rgb);
auto* cloud_ptr = cloud.get();
scene.AddOpenGLObject("cloud", std::move(cloud));

// 3. External processing (e.g., PCL segmentation)
std::vector<size_t> selected_indices = YourAlgorithm::ProcessPointCloud();

// 4. Visualize results
gldraw::visualization::SelectionData selection;
selection.selection_name = "segmentation_result";
selection.point_indices = selected_indices;
selection.highlight_color = glm::vec3(1.0f, 0.0f, 0.0f);  // Red
gldraw::visualization::SelectionVisualizer::CreateHighlight(selection, *cloud_ptr);
```

### External Processing Integration
```cpp
// External algorithm produces results
auto surface_result = MyPlaneDetection::Extract(point_cloud);

// Convert to gldraw format
gldraw::visualization::SurfaceData viz_data;
viz_data.vertices = surface_result.vertices;
viz_data.triangle_indices = surface_result.triangles;
viz_data.color = glm::vec3(0.8f, 0.6f, 0.9f);
viz_data.show_normals = true;

// Visualize
auto surface_mesh = gldraw::visualization::SurfaceVisualizer::CreateMesh(viz_data);
scene_manager.AddObject("detected_plane", std::move(surface_mesh));
```

---

## Implementation Guidelines

### Code Quality Standards
- **Style**: Google C++ Style Guide
- **Testing**: Minimum 80% code coverage
- **Documentation**: Comprehensive API documentation
- **Performance**: Real-time interaction capabilities
- **Memory**: RAII and efficient resource management

### File Organization
- **Headers**: Clean public APIs in `include/gldraw/`
- **Implementation**: Internal details in `src/`
- **Tests**: Comprehensive coverage in `test/`
- **Examples**: Usage demonstrations in `examples/`

### Incremental Development
- **Small Steps**: Each phase independently testable
- **No Breaking Changes**: Until final reorganization phase
- **Backward Compatibility**: Existing code continues to work
- **Clear Migration Path**: Documented upgrade procedures

---

## Future Vision

### Long-term Goals
- **Industry Standard**: De facto visualization library for robotics point clouds
- **Plugin Architecture**: Easy integration with ROS, Open3D, CloudCompare
- **Real-time Performance**: Interactive visualization of multi-million point datasets
- **Rich Ecosystem**: Community-contributed visualizers and tools

### Success Metrics
- **Performance**: 60fps with 1M+ points
- **Usability**: 5-line integration for common use cases
- **Adoption**: Usage in major robotics frameworks
- **Community**: Active contributor ecosystem

---

## Next Actions

**Immediate (This Week)**:
1. Implement `SelectionVisualizer::CreateHighlight()`
2. Create integration test with real point cloud data
3. Update documentation with new API patterns

**Short-term (This Month)**:
1. Complete Phase 3 visualizers
2. Add surface visualization capabilities
3. Begin scene manager integration

**Medium-term (Next Quarter)**:
1. Enhanced data contracts for clustering/trajectories
2. Performance optimization for large datasets
3. Advanced measurement and annotation tools

This roadmap provides a clear path toward a modern, efficient, and easy-to-use point cloud visualization library that serves as the foundation for advanced robotics applications.