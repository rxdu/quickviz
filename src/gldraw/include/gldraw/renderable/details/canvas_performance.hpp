/**
 * @file canvas_performance.hpp
 * @author Claude Code Assistant
 * @date 2025-01-04
 * @brief Performance monitoring and configuration for Canvas rendering
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef OPENGL_RENDERER_CANVAS_PERFORMANCE_HPP
#define OPENGL_RENDERER_CANVAS_PERFORMANCE_HPP

#include <cstdint>
#include <cstddef>
#include <algorithm>
#include <limits>

namespace quickviz {

/**
 * @brief Comprehensive rendering statistics for performance monitoring
 * 
 * Tracks frame timing, memory usage, batching efficiency, and OpenGL
 * resource usage to help optimize Canvas rendering performance.
 */
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
  float min_frame_time_ms = std::numeric_limits<float>::max();
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
  
  /**
   * @brief Reset per-frame statistics (preserves cumulative data)
   */
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
  
  /**
   * @brief Update frame timing statistics with exponential smoothing
   * @param frame_time_ms Time taken for the current frame in milliseconds
   */
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
  
  /**
   * @brief Update statistics for individual operations
   * @param operation_time_ms Time taken for a single operation in milliseconds
   */
  void UpdateOperationStats(float operation_time_ms) {
    // Track individual operation timing (for non-batched operations)
    last_frame_time_ms += operation_time_ms; // Add to total frame time
  }
  
  // Convenience methods
  float GetFPS() const { return last_frame_time_ms > 0 ? 1000.0f / last_frame_time_ms : 0.0f; }
  float GetAvgFPS() const { return avg_frame_time_ms > 0 ? 1000.0f / avg_frame_time_ms : 0.0f; }
  size_t GetTotalMemoryMB() const { return total_memory_used / (1024 * 1024); }
};

/**
 * @brief Configuration settings for Canvas performance tuning
 * 
 * Controls batching behavior, memory management, tessellation quality,
 * and performance monitoring options.
 */
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

} // namespace quickviz

#endif /* OPENGL_RENDERER_CANVAS_PERFORMANCE_HPP */