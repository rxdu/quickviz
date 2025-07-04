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

#include <glm/glm.hpp>

#include "renderer/interface/opengl_object.hpp"
#include "renderer/shader_program.hpp"
#include "renderer/renderable/types.hpp"
#include "renderer/renderable/details/canvas_batching.hpp"
#include "renderer/renderable/details/canvas_performance.hpp"

// Forward declarations for render strategies
namespace quickviz {
class RenderStrategy;
class BatchedRenderStrategy;
class IndividualRenderStrategy;
class ShapeRenderer;
}

namespace quickviz {
// Forward declaration of Point struct
struct Point;
struct CanvasData;

// Note: LineBatch and ShapeBatch moved to details/canvas_batching.hpp

class Canvas : public OpenGlObject {
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

  // Clear all points from the canvas
  void Clear();

  // Performance and rendering methods
  void SetBatchingEnabled(bool enabled);
  bool IsBatchingEnabled() const { return batching_enabled_; }
  void FlushBatches(); // Force immediate rendering of all batches
  
  // Performance monitoring (moved to details/canvas_performance.hpp)
  const RenderStats& GetRenderStats() const;
  void ResetRenderStats();
  
  // Performance tuning and memory optimization (moved to details/canvas_performance.hpp)
  void SetPerformanceConfig(const PerformanceConfig& config);
  const PerformanceConfig& GetPerformanceConfig() const;
  
  // Memory optimization methods
  void OptimizeMemory();                    // Trigger memory optimization pass
  void PreallocateMemory(size_t estimated_objects); // Pre-allocate for known workloads
  void ShrinkToFit();                       // Release unused memory
  size_t GetMemoryUsage() const;            // Get current memory usage in bytes

  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return primitive_vao_ != 0; }

 private:
  // Load and setup background image
  void SetupBackgroundImage(int width, int height, int channels,
                            unsigned char* data);

  // Process pending updates
  void ProcessPendingUpdates();

  // Structure to hold pending updates
  struct PendingUpdate {
    enum class Type {
      kPoint,
      kLine,
      kRectangle,
      kCircle,
      kEllipse,
      kPolygon,
      kClear
    };

    Type type;
    glm::vec4 color;
    float thickness;
    LineType line_type;
    bool filled;
    
    // Command-specific parameters
    struct ellipse_params {
      float x, y, rx, ry, angle, start_angle, end_angle;
    };

    union {
      struct {  // Point parameters
        float x, y;
      } point;

      struct {  // Line parameters
        float x1, y1, x2, y2;
      } line;

      struct {  // Rectangle parameters
        float x, y, width, height;
      } rect;

      struct {  // Circle parameters
        float x, y, radius;
      } circle;

      ellipse_params ellipse;
    };
    
    // Polygon vertices (can't be in union)
    std::vector<glm::vec2> polygon_vertices;
  };

  // Background image texture
  std::mutex background_mutex_;
  std::atomic<uint32_t> background_texture_{0};

  // Background rendering gpu resources
  uint32_t background_vao_ = 0;
  uint32_t background_vbo_ = 0;
  ShaderProgram background_shader_;

  // Thread-safe data structures
  std::mutex data_mutex_;
  std::queue<PendingUpdate> pending_updates_;
  std::atomic<bool> has_pending_updates_{false};
  std::unique_ptr<CanvasData> data_;

  // Primitive rendering gpu resources
  uint32_t primitive_vao_ = 0;
  uint32_t primitive_vbo_ = 0;
  ShaderProgram primitive_shader_;

  // Batching-related members
  bool batching_enabled_ = true;  // Re-enabled, ellipse/polygon renderMode fixed
  LineBatch line_batch_;
  ShapeBatch filled_shape_batch_;
  ShapeBatch outline_shape_batch_;

  // Batch management methods
  void InitializeBatches();
  void ClearBatches();
  void UpdateBatches();
  void RenderBatches(const glm::mat4& projection, const glm::mat4& view, 
                     const glm::mat4& coord_transform);
  
  // Individual shape rendering for non-batched shapes
  void RenderIndividualShapes(const CanvasData& data, const glm::mat4& projection, 
                             const glm::mat4& view, const glm::mat4& coord_transform);
  
  // Shape generation helpers
  void GenerateCircleVertices(float cx, float cy, float radius, int segments,
                             std::vector<float>& vertices, std::vector<uint32_t>& indices,
                             bool filled, uint32_t base_index);
  void GenerateRectangleVertices(float x, float y, float width, float height,
                                std::vector<float>& vertices, std::vector<uint32_t>& indices,
                                bool filled, uint32_t base_index);
  void GenerateEllipseVertices(const PendingUpdate::ellipse_params& ellipse,
                               std::vector<float>& vertices, std::vector<uint32_t>& indices,
                               bool filled, uint32_t base_index);
  void GeneratePolygonVertices(const std::vector<glm::vec2>& points,
                               std::vector<float>& vertices, std::vector<uint32_t>& indices,
                               bool filled, uint32_t base_index);

  // Performance monitoring
  RenderStats render_stats_;

  // Performance tuning and memory optimization
  PerformanceConfig perf_config_;
  
  // Render strategy system (refactored from monolithic OnDraw)
  RenderStrategy* current_render_strategy_;
  std::unique_ptr<BatchedRenderStrategy> batched_strategy_;
  std::unique_ptr<IndividualRenderStrategy> individual_strategy_;
  
  // Unified shape renderer (Phase 2 refactoring)
  std::unique_ptr<ShapeRenderer> shape_renderer_;
  
  // Helper method to select appropriate render strategy
  RenderStrategy* SelectRenderStrategy(const CanvasData& data);

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
    
    void Start() {
      start_time = std::chrono::high_resolution_clock::now();
    }
    
    float ElapsedMs() const {
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
      return duration.count() / 1000.0f;
    }
  };
  
  mutable PerformanceTimer frame_timer_;
  mutable PerformanceTimer operation_timer_;
};
}  // namespace quickviz

#endif /* OPENGL_RENDERER_CANVAS_HPP */
