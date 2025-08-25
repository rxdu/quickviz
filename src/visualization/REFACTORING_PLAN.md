# Visualization Module Refactoring Plan

## Overview
This document outlines structural improvements for the visualization module to enhance maintainability, extensibility, and consistency.

## Phase 1: Core Infrastructure (Priority: HIGH)

### 1.1 Base Interfaces
Create fundamental interfaces that all components will implement:

```cpp
// interface/data_contract.hpp
namespace quickviz::visualization {
class DataContract {
public:
  virtual ~DataContract() = default;
  virtual bool Validate() const = 0;
  virtual std::string GetType() const = 0;
  virtual size_t GetDataSize() const = 0;
};
}

// interface/renderable_converter.hpp
template<typename DataT>
class RenderableConverter {
public:
  virtual ~RenderableConverter() = default;
  virtual std::unique_ptr<OpenGlObject> Convert(const DataT& data) = 0;
  virtual bool CanConvert(const DataContract& data) const = 0;
};
```

### 1.2 Namespace Reorganization
Establish consistent namespace hierarchy:
```
quickviz::visualization::
├── contracts::      # Data contracts
├── converters::     # Data to renderable converters
├── selection::      # Selection algorithms and state
├── spatial::        # Spatial indexing and queries
├── pipeline::       # Processing pipelines
├── helpers::        # Utility functions
└── pcl_bridge::     # PCL integration (optional)
```

## Phase 2: Data Contracts (Priority: HIGH)

### 2.1 Core Point Cloud Contracts
```cpp
// contracts/point_cloud_data.hpp
struct PointCloudData : DataContract {
  std::vector<glm::vec3> positions;
  std::optional<std::vector<glm::vec3>> colors;
  std::optional<std::vector<glm::vec3>> normals;
  std::optional<std::vector<float>> intensities;
  
  ColorMode color_mode = ColorMode::kStatic;
  float point_size = 3.0f;
  glm::vec3 default_color{0.5f, 0.5f, 1.0f};
};

// contracts/normal_data.hpp
struct NormalData : DataContract {
  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> normals;
  float arrow_length = 0.1f;
  glm::vec3 color{0.0f, 1.0f, 0.0f};
};
```

### 2.2 Robotics-Specific Contracts
```cpp
// contracts/pose_data.hpp
struct PoseData : DataContract {
  glm::vec3 position;
  glm::quat orientation;
  float frame_size = 1.0f;
  bool show_trail = false;
  std::vector<glm::vec3> trail_points;
};

// contracts/path_data.hpp
struct PathData : DataContract {
  std::vector<glm::vec3> waypoints;
  PathInterpolation interpolation = PathInterpolation::kLinear;
  bool show_direction = true;
  ColorGradient color_gradient;
};
```

## Phase 3: Selection System Refactoring (Priority: MEDIUM)

### 3.1 Split PointCloudSelector
Current monolithic class should be split into:

```
selection/
├── core/
│   ├── selector_base.hpp         # Abstract base class
│   ├── selection_state.hpp       # Selection state management
│   └── selection_manager.hpp     # Multi-selection coordination
├── algorithms/
│   ├── ray_picker.hpp           # Ray-based picking
│   ├── region_selector.hpp      # Box, sphere, cylinder selection
│   ├── lasso_selector.hpp       # 2D lasso selection
│   └── plane_selector.hpp       # Plane-based selection
├── point_cloud_selector.hpp     # Composed selector using algorithms
└── mesh_selector.hpp            # Future: mesh selection
```

### 3.2 Selection Pipeline
```cpp
// selection/pipeline/selection_pipeline.hpp
class SelectionPipeline {
  void AddFilter(SelectionFilter filter);
  void AddProcessor(SelectionProcessor processor);
  SelectionResult Execute(const SelectionInput& input);
};
```

## Phase 4: Spatial Indexing Abstraction (Priority: LOW)

### 4.1 Unified Spatial Query Interface
```cpp
// spatial/spatial_index.hpp
template<typename PointT>
class SpatialIndex {
public:
  virtual void Build(const std::vector<PointT>& points) = 0;
  virtual std::vector<size_t> RadiusSearch(const PointT& center, float radius) = 0;
  virtual std::optional<size_t> NearestNeighbor(const PointT& query) = 0;
};

// spatial/kdtree_index.hpp
template<typename PointT>
class KdTreeIndex : public SpatialIndex<PointT> {
  // PCL KdTree wrapper implementation
};
```

## Phase 5: Processing Pipelines (Priority: LOW)

### 5.1 Composable Processing
```cpp
// pipeline/processing_step.hpp
template<typename InputT, typename OutputT>
class ProcessingStep {
  virtual OutputT Process(const InputT& input) = 0;
};

// pipeline/processing_pipeline.hpp
class ProcessingPipeline {
  template<typename StepT>
  void AddStep(StepT step);
  
  void Execute();
};
```

## Implementation Order

### Immediate (This Week):
1. Create base interfaces (DataContract, RenderableConverter)
2. Add missing data contracts for existing renderables
3. Reorganize namespaces consistently

### Short Term (This Month):
1. Refactor PointCloudSelector into smaller components
2. Create proper converter classes for each data contract
3. Add comprehensive unit tests

### Long Term (This Quarter):
1. Implement spatial indexing abstraction
2. Create processing pipeline framework
3. Add performance benchmarks

## Benefits

1. **Maintainability**: Clear separation of concerns and single responsibility
2. **Extensibility**: Easy to add new data types and converters
3. **Testability**: Smaller, focused classes are easier to test
4. **Performance**: Can optimize individual components independently
5. **Documentation**: Clear interfaces make API usage obvious

## Migration Strategy

1. **Backward Compatibility**: Keep existing classes during transition
2. **Incremental Migration**: Migrate one component at a time
3. **Deprecation Warnings**: Mark old APIs as deprecated
4. **Documentation**: Update docs with migration guides

## Testing Requirements

Each refactored component needs:
- Unit tests with >80% coverage
- Integration tests for component interactions
- Performance benchmarks for critical paths
- Example usage in demo applications