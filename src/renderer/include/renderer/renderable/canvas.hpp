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

namespace quickviz {
// Forward declaration of Point struct
struct Point;
struct CanvasData;

// Batched rendering structures for improved performance
struct LineBatch {
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec4> colors;
  std::vector<float> thicknesses;
  std::vector<LineType> line_types;
  uint32_t vao = 0;
  uint32_t position_vbo = 0;
  uint32_t color_vbo = 0;
  bool needs_update = true;
};

struct ShapeBatch {
  std::vector<float> vertices;
  std::vector<uint32_t> indices;
  std::vector<glm::vec4> colors;
  uint32_t vao = 0;
  uint32_t vertex_vbo = 0;
  uint32_t color_vbo = 0;
  uint32_t ebo = 0;
  bool needs_update = true;
};

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
  void SetBatchingEnabled(bool enabled) { batching_enabled_ = enabled; }
  bool IsBatchingEnabled() const { return batching_enabled_; }
  void FlushBatches(); // Force immediate rendering of all batches
  
  // Performance monitoring
  struct RenderStats {
    // Rendering statistics
    uint32_t points_rendered = 0;
    uint32_t lines_rendered = 0;
    uint32_t shapes_rendered = 0;
    uint32_t draw_calls = 0;
    uint32_t state_changes = 0;
    
    // Timing statistics
    float last_frame_time_ms = 0.0f;
    float avg_frame_time_ms = 0.0f;
    float min_frame_time_ms = 999999.0f;
    float max_frame_time_ms = 0.0f;
    uint32_t frame_count = 0;
    
    // Memory statistics
    size_t vertex_memory_used = 0;
    size_t index_memory_used = 0;
    size_t texture_memory_used = 0;
    size_t total_memory_used = 0;
    
    // Batching efficiency
    uint32_t batched_objects = 0;
    uint32_t individual_objects = 0;
    float batch_efficiency = 0.0f; // Percentage of objects that were batched
    
    // OpenGL resource usage
    uint32_t active_vaos = 0;
    uint32_t active_vbos = 0;
    uint32_t active_textures = 0;
    
    void Reset() {
      points_rendered = 0;
      lines_rendered = 0; 
      shapes_rendered = 0;
      draw_calls = 0;
      state_changes = 0;
      last_frame_time_ms = 0.0f;
      // Note: Don't reset cumulative stats like avg, min, max, frame_count
      vertex_memory_used = 0;
      index_memory_used = 0;
      texture_memory_used = 0;
      total_memory_used = 0;
      batched_objects = 0;
      individual_objects = 0;
      batch_efficiency = 0.0f;
      active_vaos = 0;
      active_vbos = 0;
      active_textures = 0;
    }
    
    void UpdateFrameStats(float frame_time_ms) {
      last_frame_time_ms = frame_time_ms;
      frame_count++;
      
      // Update min/max
      min_frame_time_ms = std::min(min_frame_time_ms, frame_time_ms);
      max_frame_time_ms = std::max(max_frame_time_ms, frame_time_ms);
      
      // Update rolling average (with decay factor)
      const float alpha = 0.1f; // Smoothing factor
      avg_frame_time_ms = avg_frame_time_ms * (1.0f - alpha) + frame_time_ms * alpha;
      
      // Calculate batch efficiency
      uint32_t total_objects = batched_objects + individual_objects;
      batch_efficiency = total_objects > 0 ? 
        (static_cast<float>(batched_objects) / total_objects) * 100.0f : 0.0f;
    }
    
    void UpdateOperationStats(float operation_time_ms) {
      // Track individual operation timing (for non-batched operations)
      last_frame_time_ms += operation_time_ms; // Add to total frame time
    }
    
    // Convenience methods
    float GetFPS() const { return last_frame_time_ms > 0 ? 1000.0f / last_frame_time_ms : 0.0f; }
    float GetAvgFPS() const { return avg_frame_time_ms > 0 ? 1000.0f / avg_frame_time_ms : 0.0f; }
    size_t GetTotalMemoryMB() const { return total_memory_used / (1024 * 1024); }
  };
  
  const RenderStats& GetRenderStats() const { return render_stats_; }
  void ResetRenderStats() { render_stats_.Reset(); }
  
  // Performance tuning and memory optimization
  struct PerformanceConfig {
    // Batching configuration
    bool auto_batching_enabled = true;
    size_t max_batch_size = 10000;           // Maximum objects per batch
    size_t batch_resize_threshold = 5000;     // When to resize batch buffers
    
    // Memory management
    bool object_pooling_enabled = true;
    size_t initial_pool_size = 1000;         // Initial pool size for reusable objects
    size_t max_pool_size = 50000;            // Maximum pool size
    bool aggressive_memory_cleanup = false;   // Clean up unused memory more aggressively
    
    // Performance monitoring
    bool detailed_timing_enabled = false;    // Enable detailed per-operation timing
    bool memory_tracking_enabled = true;     // Track memory usage
    size_t stats_update_frequency = 60;      // Update stats every N frames
    
    // Quality vs Performance trade-offs
    int circle_segments = 32;                // Default circle tessellation
    int ellipse_segments = 32;               // Default ellipse tessellation
    bool adaptive_tessellation = true;       // Adjust tessellation based on size
    float tessellation_scale_factor = 50.0f; // Pixels per segment for adaptive mode
  };
  
  void SetPerformanceConfig(const PerformanceConfig& config) { perf_config_ = config; }
  const PerformanceConfig& GetPerformanceConfig() const { return perf_config_; }
  
  // Memory optimization methods
  void OptimizeMemory();                    // Trigger memory optimization pass
  void PreallocateMemory(size_t estimated_objects); // Pre-allocate for known workloads
  void ShrinkToFit();                       // Release unused memory
  size_t GetMemoryUsage() const;            // Get current memory usage in bytes

  void AllocateGpuResources() override;
  void ReleaseGpuResources() override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;

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
      
      struct {  // Ellipse parameters
        float x, y, rx, ry, angle, start_angle, end_angle;
      } ellipse;
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

  // Performance monitoring
  RenderStats render_stats_;

  // Performance tuning and memory optimization
  PerformanceConfig perf_config_;
  
  // Object pooling for memory optimization
  template<typename T>
  struct ObjectPool {
    std::vector<std::unique_ptr<T>> available_objects;
    std::vector<std::unique_ptr<T>> used_objects;
    size_t total_created = 0;
    size_t peak_usage = 0;
    
    T* Acquire() {
      if (available_objects.empty()) {
        available_objects.push_back(std::make_unique<T>());
        total_created++;
      }
      
      auto obj = std::move(available_objects.back());
      available_objects.pop_back();
      T* ptr = obj.get();
      used_objects.push_back(std::move(obj));
      
      peak_usage = std::max(peak_usage, used_objects.size());
      return ptr;
    }
    
    void Release(T* obj) {
      auto it = std::find_if(used_objects.begin(), used_objects.end(),
        [obj](const std::unique_ptr<T>& ptr) { return ptr.get() == obj; });
      
      if (it != used_objects.end()) {
        available_objects.push_back(std::move(*it));
        used_objects.erase(it);
      }
    }
    
    void ShrinkToFit(size_t max_size) {
      while (available_objects.size() > max_size) {
        available_objects.pop_back();
      }
    }
    
    size_t GetMemoryUsage() const {
      return (available_objects.size() + used_objects.size()) * sizeof(T) + 
             available_objects.capacity() * sizeof(std::unique_ptr<T>) +
             used_objects.capacity() * sizeof(std::unique_ptr<T>);
    }
  };
  
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
