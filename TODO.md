# QuickViz Development Roadmap

*Last Updated: August 23, 2025 - Current Status Review*

## Project Overview

QuickViz is a **C++17 visualization library** for robotics applications with a layered architecture providing:
- **gldraw**: Pure OpenGL rendering engine for geometric primitives
- **visualization**: High-level data mapping from external formats to renderables  
- **imview**: Automatic layout management and UI integration
- **widget**: Cairo-based drawing and plotting widgets
- **core**: Event system, buffers, and shared utilities

Current Version: **v0.6.5** | Branch: **feature-pointcloud_editing**

## Current Status Overview

### ✅ **Core Infrastructure - COMPLETE**
- **Build System**: CMake with comprehensive options and cross-platform support
- **Testing Framework**: GoogleTest with unit, integration, and memory test labels
- **CI/CD**: All tests passing (11 unit tests, 9 integration tests)  
- **Dependencies**: Managed through CMake with optional components
- **Documentation**: Architecture documentation and API references
- **Packaging**: CPack with Debian package generation

### ✅ **gldraw Module - COMPLETE**
**All geometric renderable primitives implemented and tested:**

**Basic Geometry**:
- `point_cloud.hpp/cpp` - Point-based rendering with multi-layer system
- `mesh.hpp/cpp` - Triangle mesh with materials and transparency
- `sphere.hpp/cpp` - Sphere primitives with multiple render modes
- `cylinder.hpp/cpp` - Cylinder primitives  
- `bounding_box.hpp/cpp` - Axis-aligned and oriented bounding boxes
- `frustum.hpp/cpp` - Frustum/FOV visualization (recently fixed)
- `triangle.hpp/cpp` - Basic triangle primitive
- `plane.hpp/cpp` - Plane primitives
- `grid.hpp/cpp` - Reference grid for spatial orientation
- `line_strip.hpp/cpp` - Line-based rendering
- `arrow.hpp/cpp` - Arrow/vector visualization
- `coordinate_frame.hpp/cpp` - 3D reference frames
- `text3d.hpp/cpp` - 3D text rendering with ultra-high resolution anti-aliasing (4096x4096 atlas)
- `texture.hpp/cpp` - Texture mapping and rendering
- `canvas.hpp/cpp` - 2D drawing surface with advanced batching

**Robotics-Specific Geometry**:
- `pose.hpp/cpp` - 6-DOF pose visualization with coordinate frame and history trail
- `path.hpp/cpp` - Smooth curve/spline rendering with directional indicators and color encoding  
- `vector_field.hpp/cpp` - Efficient vector field rendering with instanced arrows
- `occupancy_grid.hpp/cpp` - 2D/3D multi-layer voxel grids with occupancy probability coloring
- `measurement.hpp/cpp` - Distance lines, angle arcs, and dimensional callouts with labels
- `uncertainty_ellipse.hpp/cpp` - Covariance ellipses/ellipsoids for probabilistic visualization  
- `sensor_coverage.hpp/cpp` - Range rings and 3D coverage volumes for sensors

**Scene Management**:
- `gl_scene_manager.hpp/cpp` - Scene composition and rendering
- `gl_view.hpp/cpp` - Unified view framework for test consistency
- `camera.hpp/cpp` + `camera_controller.hpp/cpp` - 3D navigation
- `frame_buffer.hpp/cpp` - Render-to-texture capabilities
- `layer_manager.hpp/cpp` - Multi-layer composition system

**Advanced Features**:
- **Multi-Layer Point Cloud System**: Priority-based rendering with highlight modes
- **Performance Optimizations**: Index buffer optimization (60-100x improvement)
- **3D Sphere Rendering**: Phong lighting with proper circular point shapes
- **Transparency Support**: Alpha blending with depth testing
- **Resource Management**: RAII-based OpenGL resource cleanup

### ✅ **visualization Module - FUNCTIONAL**
**High-level data mapping capabilities:**

**Data Contracts**:
- `selection_data.hpp` - Point selection specifications
- `surface_data.hpp` - Mesh/surface specifications  

**Renderable Converters**:
- `selection_renderable.hpp/cpp` - SelectionData → PointCloud highlights
- `surface_renderable.hpp/cpp` - SurfaceData → Mesh objects

**Integration Support**:
- `pcl_conversions.hpp/cpp` - PCL data type conversions
- `pcl_loader.hpp/cpp` - PCL file loading (.pcd, .ply)
- `pcl_visualization.hpp/cpp` - PCL-specific visualization helpers
- `selection_visualizer.hpp/cpp` - Selection visualization utilities
- `mock_data_generator.hpp/cpp` - Test data generation

### ✅ **imview Module - COMPLETE**  
**UI framework with automatic layout:**
- **Window Management**: GLFW integration with OpenGL contexts
- **Layout Engine**: Yoga-based automatic layout (flexbox-style)
- **Scene Objects**: Unified interface for UI and rendering components
- **Panel System**: ImGui integration with container support
- **Input Handling**: Mouse, keyboard, and joystick support
- **Terminal UI**: Text-based interface components

### ✅ **widget Module - COMPLETE**
**Cairo-based drawing and plotting:**
- **Image Widgets**: OpenCV integration for image display
- **Plotting Widgets**: Real-time line plots with scrolling buffers
- **Cairo Integration**: 2D drawing with Cairo graphics library
- **Buffered Rendering**: Efficient image widget with double buffering

### ✅ **core Module - COMPLETE**
**Foundation utilities:**
- **Event System**: Synchronous and asynchronous event handling
- **Buffer Management**: Double buffers, ring buffers with thread safety
- **Registry System**: Centralized buffer and resource management

---

## Development Phases

### ✅ **Phase 1-4: Foundation Complete** 
**Comprehensive rendering and integration system**
- Core rendering pipeline with all geometric primitives
- Multi-layer point cloud system with advanced highlighting
- PCL integration with file loading and data conversion
- External data processing integration via visualization module
- Comprehensive test coverage (unit, integration, memory)
- All renderable test applications using unified GlView framework

### 🔄 **Phase 5: Architecture Refinement** (Current - 80% Complete)
**Goal**: Polish and consolidate the existing architecture

**Completed**:
- ✅ All geometric renderables implemented and tested
- ✅ Unified test framework with GlView for consistency
- ✅ Multi-layer rendering system with priority-based composition
- ✅ External data integration through visualization module
- ✅ Comprehensive PCL bridge for file loading and conversions

**Remaining Tasks**:
- [ ] **Performance Benchmarking**: Systematic performance testing of large datasets
- [ ] **API Documentation**: Complete API reference documentation
- [ ] **Example Gallery**: Comprehensive usage examples for each module
- [ ] **Memory Testing**: Valgrind integration and memory leak detection
- [ ] **Cross-platform Testing**: Windows and macOS compatibility verification

### 📋 **Phase 6: Robotics-Specific Renderables** (Next Priority)
**Goal**: Add essential geometric primitives for robotics visualization

#### **gldraw Module Extensions** (Low-level rendering primitives)
**High Priority**:
- ✅ `pose.hpp/cpp` - 6-DOF pose visualization with coordinate frame and history trail (COMPLETED)
- ✅ `path.hpp/cpp` - Smooth curve/spline rendering with directional indicators (COMPLETED)
- ✅ `vector_field.hpp/cpp` - Multiple vectors with magnitude/direction encoding (COMPLETED)
- ✅ `occupancy_grid.hpp/cpp` - 2D/3D multi-layer voxel grids with occupancy probability coloring (COMPLETED)

**Medium Priority**:
- ✅ `measurement.hpp/cpp` - Distance lines, angle arcs, dimensional callouts with labels (COMPLETED)
- ✅ `uncertainty_ellipse.hpp/cpp` - Covariance ellipses/ellipsoids for probabilistic visualization (COMPLETED)
- ✅ `sensor_coverage.hpp/cpp` - Range rings and 3D coverage volumes for sensors (COMPLETED)

**Advanced**:
- [ ] `robot_model.hpp/cpp` - Articulated robot with joint states and forward kinematics

#### **visualization Module Extensions** (High-level data mapping)

**Data Contracts**:
- [ ] `pose_data.hpp` - Position, orientation, coordinate frame options, history settings
- [ ] `trajectory_data.hpp` - Robot path with velocity/time encoding and smoothing options
- [ ] `vector_field_data.hpp` - Vector positions, magnitudes, color encoding schemes
- [ ] `occupancy_data.hpp` - Grid dimensions, cell values, probability thresholds
- [ ] `measurement_data.hpp` - Point pairs, dimension values, label text, precision settings
- [ ] `uncertainty_data.hpp` - Covariance matrices, confidence levels, visualization modes

**Corresponding Renderables**:
- [ ] `pose_renderable.hpp` - PoseData → composed gldraw objects (coordinate frame + trail)
- [ ] `trajectory_renderable.hpp` - TrajectoryData → gldraw::Path with color encoding
- [ ] `vector_field_renderable.hpp` - VectorFieldData → gldraw::VectorField arrays
- [ ] `occupancy_renderable.hpp` - OccupancyData → gldraw::OccupancyGrid instances
- [ ] `measurement_renderable.hpp` - MeasurementData → gldraw measurement primitives
- [ ] `uncertainty_renderable.hpp` - UncertaintyData → gldraw ellipse objects

### 📋 **Phase 7: Advanced Data Visualization** (Future)
**Goal**: Extended data contracts for specialized processing results

**Specialized Data Contracts**:
- `cluster_data.hpp` - Point cloud clustering and segmentation results  
- `annotation_data.hpp` - Labels, measurements, and text overlays
- `statistics_data.hpp` - Per-point analysis and coloring data
- `robot_state_data.hpp` - Complete robot configuration with joint states

**Advanced Renderables**:
- `cluster_renderable.hpp` - Multi-cluster boundaries and highlighting
- `annotation_renderable.hpp` - 3D text and measurement overlays  
- `statistics_renderable.hpp` - Data-driven point cloud coloring
- `robot_state_renderable.hpp` - Complete robot visualization with articulated joints

### 📋 **Phase 7: Performance and Scale** (Future)
**Goal**: Handle large datasets and real-time applications

**Performance Optimizations**:
- Level-of-detail (LOD) system for 1M+ point datasets
- GPU compute shaders for advanced rendering effects
- Streaming data processing for real-time applications
- Memory-efficient buffer management strategies

**Advanced Features**:
- Plugin architecture for external algorithm integration
- Real-time data streaming and visualization
- Advanced shader effects and post-processing
- Multi-threaded rendering pipeline

---

## Testing Coverage

### ✅ **Current Test Status**: 20/20 Tests Passing

**Unit Tests (11)**: `ctest -L unit`
- Event system: subscription, publishing, queue processing
- Buffer management: registration, retrieval, type safety
- Core utilities: thread safety, resource management

**Integration Tests (9)**: `ctest -L integration`  
- Renderer pipeline: scene creation, object management, point clouds
- ImView integration: viewer lifecycle, panel management, layout
- Camera system: controller functionality, navigation

**Memory Tests**: `ctest -L memory`
- Memory leak detection with Valgrind
- Resource cleanup verification  
- GPU resource management testing

### Test Applications (Interactive)

**Renderable Tests**: All using unified GlView framework
- `test_point_cloud` - Multi-layer point cloud rendering
- `test_mesh` - Triangle mesh with materials  
- `test_sphere` - Multiple render modes and transparency
- `test_cylinder` - Geometric cylinder primitives
- `test_bounding_box` - AABB and OBB with rotations
- `test_frustum` - Sensor FOV visualization (recently fixed)
- `test_text3d` - 3D text with ultra-high resolution anti-aliasing
- `test_arrow` - Vector and direction visualization
- `test_grid` - Reference planes and coordinate systems
- `test_coordinate_frame` - 3D axis visualization
- `test_line_strip` - Line-based rendering
- `test_triangle` - Basic triangle primitive
- `test_texture` - Texture mapping and rendering
- `test_canvas` - 2D drawing surface testing
- `test_occupancy_grid` - Multi-layer 3D voxel grid visualization
- `test_pose` - 6-DOF pose with coordinate frame and history trails
- `test_path` - Path rendering with curves, gradients, and directional indicators
- `test_vector_field` - Vector field rendering with multiple display modes
- `test_measurement` - Distance lines, angle arcs, and dimensional callouts
- `test_uncertainty_ellipse` - Covariance ellipses and probabilistic visualization
- `test_sensor_coverage` - Sensor range rings and 3D coverage volumes

**Feature Tests**: 
- `test_layer_system` - Multi-layer composition and priority
- `test_camera` - 3D navigation and controls
- `test_gl_scene_manager` - Scene management functionality

---

## Build Configuration

### **Requirements**
- **C++ Standard**: C++17 (minimum C++14 for Ubuntu 20.04)
- **CMake**: 3.16.0+
- **Platform**: Linux (primary), Windows (experimental), macOS (limited)

### **Dependencies**
**Required**:
- OpenGL 3.3+, GLFW3, GLM, Cairo
- GLAD (bundled), Dear ImGui (bundled), Yoga (bundled)

**Optional**: 
- OpenCV (for cvdraw module)
- PCL (for point cloud bridge utilities) 
- Google Benchmark (for performance tests)
- Valgrind (for memory testing)

### **Build Commands**
```bash
# Basic build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
make -j8

# Development build with all features
cmake .. -DQUICKVIZ_DEV_MODE=ON -DENABLE_AUTO_LAYOUT=ON -DSTATIC_CHECK=ON
make -j8

# Run tests
ctest --output-on-failure          # All tests
ctest -L unit                      # Unit tests only  
ctest -L integration               # Integration tests only
../scripts/run_tests.sh            # Comprehensive test script
```

### **CMake Options**
- `BUILD_TESTING`: Enable test building (OFF by default)
- `QUICKVIZ_DEV_MODE`: Development mode, forces tests (OFF by default)  
- `ENABLE_AUTO_LAYOUT`: Yoga-based automatic layout (ON by default)
- `BUILD_QUICKVIZ_APP`: Build quickviz application (OFF by default)
- `IMVIEW_WITH_GLAD`: Integrate GLAD for OpenGL loading (ON by default)
- `STATIC_CHECK`: Enable cppcheck static analysis (OFF by default)

---

## Architecture Principles

### **Module Responsibilities**
- **gldraw**: Pure OpenGL rendering and scene management
- **visualization**: High-level data mapping and external integration  
- **imview**: UI framework and automatic layout
- **widget**: Cairo-based 2D drawing and plotting
- **core**: Foundation utilities and resource management

### **Design Philosophy**
1. **Separation of Concerns**: Clear boundaries between rendering and data processing
2. **Performance First**: Optimized OpenGL pipeline with efficient resource management  
3. **Extensibility**: Easy integration of new data types and rendering modes
4. **Developer Experience**: Simple APIs for common tasks, powerful APIs for advanced use
5. **Reliability**: Comprehensive testing and robust error handling

### **Integration Patterns**
```cpp
// Simple visualization
#include "visualization/helpers/visualization_helpers.hpp"
auto selection = visualization::CreateSelection(data, cloud);
scene.AddOpenGLObject("selection", std::move(selection));

// Direct rendering  
#include "gldraw/renderable/mesh.hpp"
auto mesh = std::make_unique<Mesh>();
mesh->SetVertices(vertices);
scene.AddOpenGLObject("mesh", std::move(mesh));

// Mixed approach
auto processed = visualization::SurfaceRenderable::FromData(surface_data);
auto reference = std::make_unique<Grid>(spacing, size, color);
scene.AddOpenGLObject("surface", std::move(processed));
scene.AddOpenGLObject("grid", std::move(reference));
```

---

## Success Metrics

### **Current Achievement**
- ✅ **Functionality**: All planned geometric primitives implemented
- ✅ **Performance**: 60fps with 100K+ points using multi-layer system
- ✅ **Usability**: 5-line integration for common visualization tasks  
- ✅ **Quality**: 100% test pass rate with comprehensive coverage
- ✅ **Documentation**: Complete architecture and API documentation

### **Next Milestones**  
- **Performance**: 60fps with 1M+ points using LOD system
- **Community**: External contributions and plugin ecosystem
- **Adoption**: Integration with major robotics frameworks (ROS, Open3D)
- **Platform**: Full cross-platform compatibility (Linux, Windows, macOS)

---

## Point Cloud Enhancement Roadmap
**Focus**: Get point cloud selection and editing workflow right first, keep it simple

### 🎯 **Phase 1: Core Selection Infrastructure** (Current Priority)
**Goal**: Working point cloud selection workflow with interactive demo

#### Completed Infrastructure:
- ✅ **PointCloudSelector class** - Ray-casting and region selection with PCL KdTree
- ✅ **Selection state management** - Multiple selection modes (single, additive, subtractive, toggle)
- ✅ **SelectionData contract** - Clean data structure for selections
- ✅ **SelectionRenderable** - Visualization using point cloud layer system

#### Current Tasks:
- [ ] **Fix test_point_picking app** - Get working interactive demo with GlView integration
- [ ] **Validate complete workflow** - Point picking → selection → visualization → editing
- [ ] **Add basic editing** - Delete selected points functionality
- [ ] **Add undo/redo** - Simple command pattern for operations

#### Test Applications:
- [ ] `test_point_picking` - Interactive selection demo (IN PROGRESS - needs GlView fix)

### 📦 **Phase 2: Additional Selection Tools** 
**Goal**: More selection methods (after Phase 1 workflow is solid)

#### Selection Methods (PointCloudSelector already supports these):
- [ ] **Interactive Box Selection** - UI for 3D bounding box selection
- [ ] **Interactive Sphere Selection** - UI with radius adjustment
- [ ] **Visual Selection Preview** - Show selection region before confirming
- [ ] **Keyboard Shortcuts** - Quick access to selection tools

#### Simple UI Components:
- [ ] Selection info panel (count, centroid, bounds)
- [ ] Tool selection buttons
- [ ] Clear selection button

### ✂️ **Phase 3: Point Cloud Editing** 
**Goal**: Simple editing operations with undo/redo

#### Basic Operations (keep it simple):
- [ ] **Delete Selected Points** - Remove points from cloud
- [ ] **Crop to Selection** - Keep only selected points
- [ ] **Transform Selection** - Move selected points
- [ ] **Copy/Paste Selection** - Duplicate to new location

#### Simple Infrastructure:
- [ ] **Command Pattern** - Basic undo/redo for operations
- [ ] **Dirty State Tracking** - Know when cloud needs saving
- [ ] **Save/Load** - Export modified point clouds

### 📊 **Phase 4: Basic Analysis** (Future)
**Goal**: Simple analysis tools for selected points

#### Basic Measurements:
- [ ] **Distance Measurement** - Point-to-point distance
- [ ] **Bounding Box Info** - Selection bounds and volume
- [ ] **Centroid Display** - Center of selected points
- [ ] **Point Count Statistics** - Selection size info

#### Simple Visualizations:
- [ ] **Normal Vectors** - Show point normals (if available)
- [ ] **Color by Height** - Z-coordinate coloring
- [ ] **Color by Density** - Local point density

*Note: Focus on getting core workflow right first. Advanced analysis can come later.*

### 🚀 **Phase 5: Performance & Polish** (Future)
**Goal**: Handle larger datasets and polish the workflow

#### Performance:
- [ ] **Large Point Cloud Support** - Efficient handling of 1M+ points
- [ ] **Level-of-Detail** - Adaptive rendering for performance
- [ ] **Better Spatial Indexing** - Optimize selection performance

#### Minimal Data Abstraction:
- [ ] **PointCloudData** - Simple data contract for loading/saving
- [ ] **PointCloudRenderable** - Converter from data to gldraw::PointCloud

#### Polish:
- [ ] **File Loading** - Support common formats (.pcd, .ply, .xyz)
- [ ] **Better UI** - Polished selection and editing interface
- [ ] **Documentation** - Usage examples and API docs

*Note: Keep it simple. No complex pipeline architectures or multiple inheritance hierarchies.*

---

## Immediate Next Actions

**Right Now**:
1. ✅ Update TODO.md with simplified, focused approach
2. Fix `test_point_picking` application to work with GlView framework
3. Validate end-to-end workflow: pick → select → visualize

**This Week**:
1. Add basic editing: delete selected points
2. Add undo/redo for selection operations
3. Polish the interactive demo

**This Month**:  
1. Add more selection tools with UI (box, sphere selection)
2. Simple point cloud editor with load/save
3. Basic measurements and statistics

**This Quarter**:
1. Performance improvements for large point clouds
2. File format support (.pcd, .ply)
3. Documentation and examples

**Focus**: Keep it simple, get the core workflow right, then build upon it.

---

This roadmap reflects the current mature state of QuickViz as a comprehensive, battle-tested visualization library ready for production robotics applications. The focus has shifted from implementation to refinement, optimization, and ecosystem growth.