# GlDraw Module Refactor Plan

## Overview

This document outlines a comprehensive refactor plan for the gldraw module to make it high-quality and high-performance for critical use cases, rather than complete but low-quality. The focus is on simplifying the architecture, optimizing performance bottlenecks, and creating a lean, maintainable codebase.

## Current Architecture Analysis

### Strengths ✅
- **Clean abstractions**: Well-designed `OpenGlObject` interface with proper resource management
- **Strategy patterns**: Rendering strategies (batched vs individual) for Canvas
- **Advanced features**: Layer-based point cloud system with priorities, GPU ID-buffer picking
- **Modern OpenGL**: VAO/VBO management, shader programs, instanced rendering support

### Critical Issues ⚠️
- **Over-engineering**: Canvas.cpp (2069 lines) handles too many responsibilities
- **Memory hotspots**: Frequent `std::vector` reallocations during rendering
- **API complexity**: Multiple overlapping interfaces for similar functionality
- **Performance gaps**: Missing LOD, frustum culling, batch rendering for primitives

## Refactor Strategy

### Phase 1: Critical Performance Fixes 🔥

#### 1.1 Canvas Module Decomposition
**Current Problem**: Single 2069-line file is unmaintainable
**Solution**: Break into focused modules

```cpp
// Current: canvas.cpp (2069 lines)
// Proposed structure:
src/gldraw/renderable/canvas/
├── canvas_renderer.cpp      // Core rendering logic (~400 lines)
├── canvas_batcher.cpp       // Batching optimization (~300 lines)  
├── canvas_shaders.cpp       // Shader management (~200 lines)
├── canvas_geometry.cpp      // Shape generation (~300 lines)
└── canvas.cpp              // Public interface (~150 lines)
```

**Benefits**:
- Easier maintenance and debugging
- Focused optimization opportunities
- Better testability
- Cleaner separation of concerns

#### 1.2 Shader Performance Optimization
**Current Problem**: `TrySetUniform` uses expensive exception handling
**Solution**: Cache uniform locations at link time

```cpp
// Current: Expensive lookup + exception handling
bool TrySetUniform(const std::string& name, float value) {
  try {
    SetUniform(name, value);
    return true;
  } catch (const std::runtime_error& e) {
    return false;
  }
}

// Proposed: Cached uniform system
class CachedShaderProgram : public ShaderProgram {
private:
  std::unordered_map<std::string, int32_t> uniform_cache_;
  
public:
  void CacheUniforms() {
    // Cache all uniform locations at link time
  }
  
  void SetUniform(const std::string& name, float value) noexcept {
    auto it = uniform_cache_.find(name);
    if (it != uniform_cache_.end()) {
      glUniform1f(it->second, value);
    }
  }
};
```

#### 1.3 Memory Allocation Optimization
**Current Problem**: Runtime vector reallocations cause frame drops
**Solution**: Pre-allocated buffer pools

```cpp
// Add to OpenGlObject interface:
class OpenGlObject {
protected:
  struct BufferConfig {
    size_t vertex_capacity = 1024;
    size_t index_capacity = 1024;
    bool enable_streaming = false;
  };
  
  void PreallocateBuffers(const BufferConfig& config);
  void* GetVertexBufferSlot(size_t size);  // From pool
  void* GetIndexBufferSlot(size_t size);   // From pool
};

// Global memory manager
class RenderMemoryPool {
public:
  static RenderMemoryPool& Instance();
  void* AllocateVertices(size_t count, size_t stride);
  void* AllocateIndices(size_t count);
  void Reset(); // Called per frame
};
```

### Phase 2: API Simplification ✂️

#### 2.1 Remove Non-Critical Components

**Files to Remove**:
```cpp
// These add complexity without critical value:
- coordinate_system_transformer.hpp/.cpp  // Use direct matrices
- gl_view.hpp/.cpp                        // Redundant with GlSceneManager
- renderable/details/data_aware_render_strategy.cpp  // Over-engineered
- Some Canvas render strategies            // Keep only Batched + Individual
```

**Rationale**: 
- `coordinate_system_transformer`: Direct matrix operations are more transparent
- `gl_view`: Functionality duplicated in GlSceneManager
- Complex render strategies: Diminishing returns vs maintenance cost

#### 2.2 Consolidate Point Cloud Interface
**Current Problem**: Too many overloads for similar functionality

```cpp
// Current: 4+ overloads
void SetPoints(const std::vector<glm::vec4>& points, ColorMode color_mode);
void SetPoints(std::vector<glm::vec4>&& points, ColorMode color_mode);
void SetPoints(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& colors);
void SetPoints(std::vector<glm::vec3>&& points, std::vector<glm::vec3>&& colors);

// Proposed: Single template interface with perfect forwarding
template<typename PointContainer>
void SetPoints(PointContainer&& points, ColorMode mode = ColorMode::kStatic) {
  static_assert(is_point_container_v<PointContainer>);
  SetPointsImpl(std::forward<PointContainer>(points), mode);
}

template<typename PointContainer, typename ColorContainer>
void SetPoints(PointContainer&& points, ColorContainer&& colors) {
  static_assert(is_point_container_v<PointContainer>);
  static_assert(is_color_container_v<ColorContainer>);
  SetPointsImpl(std::forward<PointContainer>(points), 
                std::forward<ColorContainer>(colors));
}
```

#### 2.3 Unified Primitive Interface
**Current Problem**: Each primitive (sphere, cylinder, etc.) has different APIs
**Solution**: Consistent interface pattern

```cpp
// Base class for all geometric primitives
class GeometricPrimitive : public OpenGlObject {
public:
  void SetTransform(const glm::mat4& transform);
  void SetColor(const glm::vec4& color);
  void SetMaterial(const Material& material);
  
  // Selection support (consistent across all primitives)
  bool SupportsSelection() const override { return true; }
  std::pair<glm::vec3, glm::vec3> GetBoundingBox() const override;
  void SetHighlighted(bool highlighted) override;

protected:
  glm::mat4 transform_ = glm::mat4(1.0f);
  glm::vec4 color_ = glm::vec4(0.7f, 0.7f, 0.9f, 1.0f);
  Material material_;
  bool is_highlighted_ = false;
};

// All primitives inherit consistent interface
class Sphere : public GeometricPrimitive { /* ... */ };
class Cylinder : public GeometricPrimitive { /* ... */ };
class Box : public GeometricPrimitive { /* ... */ };
```

### Phase 3: Critical Use Case Optimizations ⚡

#### 3.1 Point Cloud Performance Enhancements
**Target**: Handle 10M+ points at 60 FPS

```cpp
class PointCloud : public OpenGlObject {
public:
  // Level-of-detail for massive datasets
  struct LODConfig {
    std::vector<float> distances = {10.0f, 50.0f, 200.0f};
    std::vector<float> point_ratios = {1.0f, 0.5f, 0.1f};
    bool enable_adaptive = true;
  };
  void SetLODConfig(const LODConfig& config);
  
  // Frustum culling to skip off-screen points
  void EnableFrustumCulling(bool enable);
  void SetFrustum(const glm::mat4& view_projection);
  
  // Streaming for datasets larger than GPU memory
  void SetStreamingMode(size_t chunk_size_mb = 256);
  void StreamChunk(size_t offset, size_t count);
  
  // GPU-side selection optimization
  void PreallocateSelectionBuffers(size_t max_selected_points);
  
  // Performance monitoring
  struct PerformanceStats {
    size_t points_rendered = 0;
    size_t points_culled = 0;
    double render_time_ms = 0.0;
    size_t gpu_memory_mb = 0;
  };
  const PerformanceStats& GetStats() const;
};
```

#### 3.2 Batch Rendering System
**Target**: Reduce draw calls by 90% for multiple objects

```cpp
// New batch renderer for primitives
class PrimitiveBatcher {
public:
  // Queue primitives for batched rendering
  void AddSphere(const glm::vec3& center, float radius, const glm::vec4& color);
  void AddCylinder(const glm::vec3& base, const glm::vec3& top, float radius, const glm::vec4& color);
  void AddBox(const glm::mat4& transform, const glm::vec4& color);
  
  // Single draw call for all queued primitives of each type
  void RenderBatch(const glm::mat4& projection, const glm::mat4& view);
  
  // Performance: estimate 100+ objects -> 3-4 draw calls
  void Clear(); // Reset for next frame
  
private:
  std::vector<SphereInstance> spheres_;
  std::vector<CylinderInstance> cylinders_;
  std::vector<BoxInstance> boxes_;
  
  // GPU instancing buffers
  uint32_t sphere_instance_vbo_;
  uint32_t cylinder_instance_vbo_;
  uint32_t box_instance_vbo_;
};

// Integration with GlSceneManager
class GlSceneManager {
  PrimitiveBatcher primitive_batcher_;
  
  void RenderInsideWindow() override {
    // Render individual complex objects
    DrawOpenGLObject();
    
    // Batch render simple primitives
    primitive_batcher_.RenderBatch(projection_, view_);
    primitive_batcher_.Clear();
  }
};
```

#### 3.3 Compile-Time Shader Optimization
**Target**: Eliminate runtime shader compilation overhead

```cpp
// Build-time shader compilation
namespace CompiledShaders {
  extern const uint32_t POINT_CLOUD_VERTEX_SPIRV[];
  extern const size_t POINT_CLOUD_VERTEX_SIZE;
  
  extern const uint32_t POINT_CLOUD_FRAGMENT_SPIRV[];  
  extern const size_t POINT_CLOUD_FRAGMENT_SIZE;
  
  extern const uint32_t PRIMITIVE_VERTEX_SPIRV[];
  extern const size_t PRIMITIVE_VERTEX_SIZE;
  
  // More shaders...
}

class PrecompiledShaderProgram {
public:
  PrecompiledShaderProgram(const uint32_t* vertex_spirv, size_t vertex_size,
                           const uint32_t* fragment_spirv, size_t fragment_size);
  
  // Skip compilation, directly create from binary
  bool CreateFromBinary();
};
```

### Phase 4: Quality and Maintainability 📈

#### 4.1 Error Handling Strategy
**Current Problem**: Exceptions can cause frame drops in real-time rendering
**Solution**: Error codes for performance-critical paths

```cpp
enum class RenderResult : uint8_t {
  Success = 0,
  OutOfMemory,
  InvalidShader,
  GpuError,
  InvalidParameters
};

// Performance-critical paths use error codes
class OpenGlObject {
  virtual RenderResult OnDraw(const glm::mat4& projection, 
                             const glm::mat4& view,
                             const glm::mat4& coord_transform) noexcept = 0;
};

// Non-critical paths can still use exceptions
class ShaderProgram {
  void LinkProgram(); // Can throw for setup/initialization
};
```

#### 4.2 Resource Management (RAII)
**Current Problem**: Manual OpenGL resource cleanup is error-prone

```cpp
// RAII wrappers for all OpenGL resources
class GLBuffer {
private:
  uint32_t id_ = 0;
  size_t size_ = 0;
  GLenum target_ = GL_ARRAY_BUFFER;
  
public:
  explicit GLBuffer(GLenum target, size_t size);
  ~GLBuffer() { if (id_) glDeleteBuffers(1, &id_); }
  
  // Move-only semantics
  GLBuffer(const GLBuffer&) = delete;
  GLBuffer& operator=(const GLBuffer&) = delete;
  
  GLBuffer(GLBuffer&& other) noexcept;
  GLBuffer& operator=(GLBuffer&& other) noexcept;
  
  void Bind() const { glBindBuffer(target_, id_); }
  uint32_t GetId() const { return id_; }
  size_t GetSize() const { return size_; }
};

class GLVertexArray {
  uint32_t id_ = 0;
public:
  GLVertexArray();
  ~GLVertexArray() { if (id_) glDeleteVertexArrays(1, &id_); }
  
  // Move-only
  GLVertexArray(const GLVertexArray&) = delete;
  GLVertexArray& operator=(const GLVertexArray&) = delete;
  GLVertexArray(GLVertexArray&& other) noexcept;
  GLVertexArray& operator=(GLVertexArray&& other) noexcept;
  
  void Bind() const { glBindVertexArray(id_); }
};
```

#### 4.3 Built-in Performance Monitoring
**Solution**: Integrated profiling for optimization guidance

```cpp
class PerformanceMonitor {
public:
  struct FrameStats {
    uint64_t draw_calls = 0;
    uint64_t vertices_rendered = 0;
    uint64_t triangles_rendered = 0;
    double gpu_time_ms = 0.0;
    double cpu_time_ms = 0.0;
    size_t gpu_memory_used_mb = 0;
    size_t cpu_memory_used_mb = 0;
  };
  
  void BeginFrame();
  void EndFrame();
  void RecordDrawCall(size_t vertex_count);
  
  const FrameStats& GetCurrentFrameStats() const;
  const FrameStats& GetAverageStats(int frame_count = 60) const;
  
  // Performance warnings
  bool IsPerformanceGood() const; // < 16.67ms frame time
  std::vector<std::string> GetPerformanceWarnings() const;
};

// Integration with GlSceneManager
class GlSceneManager {
  PerformanceMonitor perf_monitor_;
  
  void Draw() override {
    perf_monitor_.BeginFrame();
    // ... rendering ...
    perf_monitor_.EndFrame();
    
    if (!perf_monitor_.IsPerformanceGood()) {
      auto warnings = perf_monitor_.GetPerformanceWarnings();
      for (const auto& warning : warnings) {
        std::cerr << "Performance warning: " << warning << std::endl;
      }
    }
  }
};
```

### Phase 5: Proposed New Architecture 🏗️

#### 5.1 Streamlined Directory Structure
```cpp
src/gldraw/
├── core/                          // Essential components only
│   ├── renderer.hpp               // Main rendering coordinator
│   ├── shader_manager.hpp         // Cached shader system
│   ├── buffer_manager.hpp         // Memory pools and RAII wrappers
│   ├── render_context.hpp         // Shared rendering state
│   └── performance_monitor.hpp    // Built-in profiling
├── objects/                       // Streamlined renderables
│   ├── point_cloud.hpp            // Optimized for large datasets
│   ├── primitives.hpp             // Sphere, cylinder, box (unified interface)
│   ├── lines.hpp                  // Line strips, paths, grids
│   └── text.hpp                   // 3D text rendering
├── selection/                     // Object + point selection
│   ├── object_picker.hpp          // Ray-casting selection
│   └── point_picker.hpp           // GPU ID-buffer picking
└── batching/                      // Batch rendering systems
    ├── primitive_batcher.hpp      // Instanced primitive rendering
    └── line_batcher.hpp           // Batched line rendering
```

#### 5.2 Core Interface Simplification
```cpp
// Single main interface for all rendering
class Renderer {
public:
  // Setup
  void Initialize();
  void SetViewport(int width, int height);
  void SetCamera(const glm::mat4& projection, const glm::mat4& view);
  
  // Object management (simplified)
  template<typename T>
  uint32_t AddObject(std::unique_ptr<T> object);
  void RemoveObject(uint32_t id);
  void ClearObjects();
  
  // Rendering
  void BeginFrame();
  void EndFrame();
  RenderResult Render() noexcept;
  
  // Selection
  uint32_t SelectObjectAt(float screen_x, float screen_y);
  std::vector<uint32_t> SelectPointsAt(float screen_x, float screen_y, float radius);
  
  // Performance
  const PerformanceMonitor::FrameStats& GetStats() const;
};
```

## Implementation Timeline

### Week 1: Critical Performance
- [ ] Split canvas.cpp into focused modules
- [ ] Implement shader uniform caching
- [ ] Add buffer pre-allocation system
- [ ] Basic performance monitoring

### Week 2: API Cleanup  
- [ ] Remove coordinate_system_transformer
- [ ] Remove gl_view module
- [ ] Consolidate PointCloud SetPoints() overloads
- [ ] Create unified GeometricPrimitive base class

### Week 3: Advanced Optimization
- [ ] Implement point cloud LOD system
- [ ] Add primitive batch rendering
- [ ] Integrate pre-compiled shaders
- [ ] Add frustum culling for point clouds

### Week 4: Quality & Testing
- [ ] RAII resource wrappers
- [ ] Error code system for critical paths
- [ ] Comprehensive performance testing
- [ ] Documentation updates

## Success Metrics

### Performance Targets
- **Point Clouds**: 10M points at 60 FPS (currently ~1M points)
- **Draw Calls**: <10 per frame for typical scenes (currently 50-100+)
- **Memory**: <500MB GPU memory for large scenes (currently unbounded)
- **Startup**: <100ms initialization (currently ~500ms)

### Code Quality Targets
- **Lines of Code**: Reduce by 30% while maintaining functionality
- **Cyclomatic Complexity**: <10 for all critical-path methods
- **Test Coverage**: >90% for core rendering paths
- **Build Time**: <30 seconds for clean build (currently ~2 minutes)

## Risk Assessment

### High Risk
- **Breaking Changes**: Significant API changes may require updates to user code
- **Performance Regressions**: Optimization attempts might initially slow things down
- **Resource Management**: RAII changes could introduce new types of bugs

### Mitigation Strategies
- **Incremental Migration**: Maintain backwards compatibility during transition
- **Benchmarking**: Continuous performance testing throughout refactor
- **Staged Rollout**: Deploy optimizations in phases with rollback capability

## Conclusion

This refactor plan transforms gldraw from a feature-complete but complex system into a high-performance, maintainable rendering engine optimized for robotics visualization workloads. The focus on critical use cases (large point clouds, interactive selection, efficient primitive rendering) will provide significant value while reducing maintenance overhead.

The key insight is that **less can be more** - by removing non-critical features and optimizing the essential components, we create a more reliable and performant system that better serves the target use cases.