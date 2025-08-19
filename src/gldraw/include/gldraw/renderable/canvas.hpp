/**
 * @file canvas.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-16
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef OPENGL_RENDERER_CANVAS_HPP
#define OPENGL_RENDERER_CANVAS_HPP

#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <atomic>
#include <queue>
#include <chrono>
#include <algorithm>
#include <unordered_map>

#include <glm/glm.hpp>

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"
#include "gldraw/renderable/types.hpp"
#include "gldraw/renderable/details/canvas_batching.hpp"
#include "gldraw/renderable/details/canvas_performance.hpp"

// Forward declarations for internal components
namespace quickviz {

// Forward declarations for shape and render context types
struct Polygon;
struct Ellipse;
struct RenderContext;

namespace internal {
class OpenGLResourcePool;
class EfficientShapeRenderer;
class CanvasDataManager;
class AdaptiveStrategySelector;
}
}

// Forward declarations for render strategies
namespace quickviz {
class RenderStrategy;
class BatchedRenderStrategy;
class IndividualRenderStrategy;
class ShapeRenderer;
}  // namespace quickviz

namespace quickviz {
// Forward declaration of Point struct
struct Point;
struct CanvasData;

// Note: LineBatch and ShapeBatch moved to details/canvas_batching.hpp

class Canvas : public OpenGlObject {
 public:
  using PerformanceConfig = quickviz::PerformanceConfig;

 public:
  Canvas();
  ~Canvas();

  // public methods
  void AddPoint(float x, float y, const glm::vec4& color,
                float thickness = 1.0f);
  void AddLine(float x1, float y1, float x2, float y2, const glm::vec4& color,
               float thickness = 1.0f, LineType line_type = LineType::kSolid);
  void AddRectangle(float x, float y, float width, float height,
                    const glm::vec4& color, bool filled = true,
                    float thickness = 1.0f,
                    LineType line_type = LineType::kSolid);
  void AddCircle(float x, float y, float radius, const glm::vec4& color,
                 bool filled = true, float thickness = 1.0f,
                 LineType line_type = LineType::kSolid);
  void AddEllipse(float x, float y, float rx, float ry, float angle,
                  float start_angle, float end_angle, const glm::vec4& color,
                  bool filled = true, float thickness = 1.0f,
                  LineType line_type = LineType::kSolid);
  void AddPolygon(const std::vector<glm::vec2>& points, const glm::vec4& color,
                  bool filled = true, float thickness = 1.0f,
                  LineType line_type = LineType::kSolid);

  void AddBackgroundImage(const std::string& image_path,
                          const glm::vec3& origin, float resolution);
  glm::vec2 GetBackgroundImageSize() const;

  // Clear all points from the canvas
  void Clear();

  // Performance and rendering methods
  void SetBatchingEnabled(bool enabled);
  bool IsBatchingEnabled() const;
  void FlushBatches();  // Force immediate rendering of all batches

  // Performance monitoring (moved to details/canvas_performance.hpp)
  const RenderStats& GetRenderStats() const;
  void ResetRenderStats();

  // Performance tuning and memory optimization (moved to
  // details/canvas_performance.hpp)
  void SetPerformanceConfig(const PerformanceConfig& config);
  const PerformanceConfig& GetPerformanceConfig() const;

  // Memory optimization methods
  void OptimizeMemory();  // Trigger memory optimization pass
  void PreallocateMemory(
      size_t estimated_objects);  // Pre-allocate for known workloads
  void ShrinkToFit();             // Release unused memory
  size_t GetMemoryUsage() const;  // Get current memory usage in bytes

  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override {
    return primitive_vao_ != 0;
  }

 private:
  // Load and setup background image
  void SetupBackgroundImage(int width, int height, int channels,
                            unsigned char* data);

  // Data management helper (delegates to data manager)
  void ProcessPendingUpdates();

  // Background image texture
  glm::vec2 background_image_size_{0.0f, 0.0f};
  std::mutex background_mutex_;
  std::atomic<uint32_t> background_texture_{0};

  // Background rendering gpu resources
  uint32_t background_vao_ = 0;
  uint32_t background_vbo_ = 0;
  ShaderProgram background_shader_;

  // Core data storage - original working system
  std::unique_ptr<CanvasData> data_;
  mutable std::mutex data_mutex_;
  
  // Pending updates system
  struct PendingUpdate {
    enum class Type {
      kPoint, kLine, kRectangle, kCircle, kEllipse, kPolygon, kClear
    };
    Type type;
    glm::vec4 color;
    float thickness;
    LineType line_type;
    bool filled;
    
    struct ellipse_params {
      float x, y, rx, ry, angle, start_angle, end_angle;
    };
    
    union {
      struct { float x, y; } point;
      struct { float x1, y1, x2, y2; } line;
      struct { float x, y, width, height; } rect;
      struct { float x, y, radius; } circle;
      ellipse_params ellipse;
    };
    
    std::vector<glm::vec2> polygon_vertices;
  };
  
  std::queue<PendingUpdate> pending_updates_;
  std::atomic<bool> has_pending_updates_{false};
  
  // Batch data structures - original working system
  std::unordered_map<LineType, LineBatch> line_batches_;
  ShapeBatch filled_shape_batch_;
  std::unordered_map<LineType, ShapeBatch> outline_shape_batches_;
  
  // Batch order tracking for preserving draw order
  BatchOrderTracker batch_order_tracker_;

  // Primitive rendering gpu resources
  uint32_t primitive_vao_ = 0;
  uint32_t primitive_vbo_ = 0;
  ShaderProgram primitive_shader_;

  // Original working render strategy system
  RenderStrategy* current_render_strategy_;
  std::unique_ptr<BatchedRenderStrategy> batched_strategy_;
  std::unique_ptr<IndividualRenderStrategy> individual_strategy_;
  std::unique_ptr<ShapeRenderer> shape_renderer_;

  // Batching configuration
  bool batching_enabled_ = true;

  // Batch management methods
  void InitializeBatches();
  void ClearBatches();
  void UpdateBatches();
  void RenderBatches(const glm::mat4& projection, const glm::mat4& view,
                     const glm::mat4& coord_transform);
  void RenderBatchesInOrder(const glm::mat4& projection, const glm::mat4& view,
                           const glm::mat4& coord_transform);

  // Individual shape rendering for non-batched shapes
  void RenderIndividualShapes(const CanvasData& data,
                              const glm::mat4& projection,
                              const glm::mat4& view,
                              const glm::mat4& coord_transform);

  // Single shape rendering methods for selective batching
  void RenderSinglePolygon(const Polygon& polygon, const RenderContext& context);
  void RenderSingleEllipse(const Ellipse& ellipse, const RenderContext& context);

  // Phase 1.2: Resource pool helper for efficient individual shape rendering
  void RenderShapeWithPool(const std::vector<float>& vertices, 
                          const glm::vec4& color, float thickness,
                          unsigned int primitive_type, LineType line_type = LineType::kSolid);
  
  // Immediate rendering methods for non-batching mode
  void RenderPointImmediate(float x, float y, const glm::vec4& color, float thickness);
  void RenderLineImmediate(float x1, float y1, float x2, float y2, const glm::vec4& color, 
                          float thickness, LineType line_type);
  void RenderRectangleImmediate(float x, float y, float width, float height,
                               const glm::vec4& color, bool filled, float thickness, 
                               LineType line_type);
  void RenderCircleImmediate(float x, float y, float radius, const glm::vec4& color,
                            bool filled, float thickness, LineType line_type);
  void RenderEllipseImmediate(float x, float y, float rx, float ry, float angle,
                             float start_angle, float end_angle, const glm::vec4& color,
                             bool filled, float thickness, LineType line_type);
  void RenderPolygonImmediate(const std::vector<glm::vec2>& points, const glm::vec4& color,
                             bool filled, float thickness, LineType line_type);
  
  // Helper to get current view/projection matrices for immediate rendering
  void GetCurrentMatrices(glm::mat4& projection, glm::mat4& view, glm::mat4& coord_transform);

  // Shape generation helpers
  void GenerateCircleVertices(float cx, float cy, float radius, int segments,
                              std::vector<float>& vertices,
                              std::vector<uint32_t>& indices, bool filled,
                              uint32_t base_index, uint32_t sequence_number = 0);
  void GenerateRectangleVertices(float x, float y, float width, float height,
                                 std::vector<float>& vertices,
                                 std::vector<uint32_t>& indices, bool filled,
                                 uint32_t base_index, uint32_t sequence_number = 0);
  void GenerateEllipseVertices(float x, float y, float rx, float ry, float angle,
                               float start_angle, float end_angle,
                               std::vector<float>& vertices,
                               std::vector<uint32_t>& indices, bool filled,
                               uint32_t base_index);
  void GeneratePolygonVertices(const std::vector<glm::vec2>& points,
                               std::vector<float>& vertices,
                               std::vector<uint32_t>& indices, bool filled,
                               uint32_t base_index);

  // Performance monitoring
  RenderStats render_stats_;

  // Performance tuning and memory optimization
  PerformanceConfig perf_config_;

  // Memory tracking
  struct MemoryTracker {
    std::atomic<size_t> current_usage{0};
    std::atomic<size_t> peak_usage{0};
    std::atomic<size_t> total_allocations{0};
    std::atomic<size_t> total_deallocations{0};

    void RecordAllocation(size_t size) {
      current_usage += size;
      total_allocations++;
      peak_usage = std::max(peak_usage.load(), current_usage.load());
    }

    void RecordDeallocation(size_t size) {
      current_usage -= size;
      total_deallocations++;
    }

    void Reset() {
      current_usage = 0;
      peak_usage = 0;
      total_allocations = 0;
      total_deallocations = 0;
    }
  };

  mutable MemoryTracker memory_tracker_;

  // Timing utilities for performance monitoring
  struct PerformanceTimer {
    std::chrono::high_resolution_clock::time_point start_time;

    void Start() { start_time = std::chrono::high_resolution_clock::now(); }

    float ElapsedMs() const {
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
          end_time - start_time);
      return duration.count() / 1000.0f;
    }
  };

  mutable PerformanceTimer frame_timer_;
  mutable PerformanceTimer operation_timer_;
};
}  // namespace quickviz

#endif /* OPENGL_RENDERER_CANVAS_HPP */
