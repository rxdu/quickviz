/**
 * @file point_cloud.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-05
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef COMPONENT_OPENGL_POINT_CLOUD_HPP
#define COMPONENT_OPENGL_POINT_CLOUD_HPP

#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <queue>

#include <glm/glm.hpp>
#include "imview/interface/opengl_object.hpp"
#include "imview/component/opengl/shader_program.hpp"

namespace quickviz {

enum class PointRenderMode { Points, Spheres };

class PointCloud : public OpenGlObject {
 public:
  PointCloud();
  ~PointCloud();

  // Data management
  enum class ColorMode {
    kStatic,       // use default color
    kHeightField,  // use z-coordinate of points as height field
    kScalarField  // use last component of points as scalar field (x,y,z,scalar)
  };

  // Buffer update strategy
  enum class BufferUpdateStrategy {
    kAuto,           // Automatically choose based on point count and size
    kBufferSubData,  // Always use glBufferSubData
    kMapBuffer       // Always use glMapBufferRange
  };

  // Thread-safe methods for setting and updating points
  void SetPoints(const std::vector<glm::vec4>& points, ColorMode color_mode);
  void SetPoints(std::vector<glm::vec4>&& points, ColorMode color_mode);

  // Optimized methods for real-time updates
  void PreallocateBuffers(size_t max_points);
  void UpdatePointSubset(const std::vector<glm::vec4>& points, size_t offset,
                         ColorMode color_mode);

  // Buffer update configuration
  void SetBufferUpdateStrategy(BufferUpdateStrategy strategy) {
    buffer_update_strategy_ = strategy;
  }
  void SetBufferUpdateThreshold(size_t threshold) {
    buffer_update_threshold_ = threshold;
  }

  // Appearance
  void SetPointSize(float size);
  void SetDefaultColor(const glm::vec3& color);
  void SetOpacity(float opacity);
  void SetScalarRange(float min_val, float max_val);
  void SetRenderMode(PointRenderMode mode);

 private:
  void AllocateGpuResources() override;
  void ReleaseGpuResources() override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view, 
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  void UpdateColors(ColorMode color_mode, size_t start_idx, size_t count);

  // Helper methods for buffer updates
  void UpdateBufferWithSubData(uint32_t buffer, const void* data,
                               size_t size_bytes, size_t offset_bytes = 0);
  void UpdateBufferWithMapping(uint32_t buffer, const void* data,
                               size_t size_bytes, size_t offset_bytes = 0);
  bool ShouldUseBufferMapping(size_t point_count) const;

  // Thread-safe data processing
  void ProcessPendingUpdates();

  // Structure to hold pending updates
  struct PendingUpdate {
    std::vector<glm::vec4> points;
    size_t offset;
    ColorMode color_mode;
    bool is_subset;
  };

  // OpenGL resources
  uint32_t vao_ = 0;
  uint32_t position_vbo_ = 0;
  uint32_t color_vbo_ = 0;
  ShaderProgram shader_;

  // Thread-safe data structures
  std::mutex data_mutex_;
  std::queue<PendingUpdate> pending_updates_;
  std::atomic<bool> has_pending_update_{false};

  // Thread-safe appearance settings
  std::mutex appearance_mutex_;
  std::atomic<float> point_size_{3.0f};
  glm::vec3 default_color_ = glm::vec3(0.25f, 0.0f, 1.0f);
  std::atomic<float> opacity_{1.0f};
  std::atomic<float> min_scalar_{0.0f};
  std::atomic<float> max_scalar_{1.0f};
  std::atomic<PointRenderMode> render_mode_{PointRenderMode::Points};

  // Data for OpenGL operations (only accessed in OnDraw)
  std::vector<glm::vec3> points_;
  std::vector<glm::vec3> colors_;

  // Buffer management
  size_t buffer_capacity_ = 0;
  size_t active_points_ = 0;
  bool buffers_preallocated_ = false;
  BufferUpdateStrategy buffer_update_strategy_ = BufferUpdateStrategy::kAuto;
  size_t buffer_update_threshold_ = 10000;  // Default threshold: 10,000 points

  bool needs_update_ = false;
};
}  // namespace quickviz

#endif /* COMPONENT_OPENGL_POINT_CLOUD_HPP */
