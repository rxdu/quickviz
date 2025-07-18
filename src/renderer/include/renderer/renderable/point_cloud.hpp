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

#include <glm/glm.hpp>

#include "renderer/interface/opengl_object.hpp"
#include "renderer/shader_program.hpp"
#include "renderer/renderable/types.hpp"

namespace quickviz {
class PointCloud : public OpenGlObject {
 public:
  PointCloud();
  ~PointCloud();

  // Data management
  enum class ColorMode {
    kStatic,       // use default color
    kHeightField,  // use z-coordinate of points as height field
    kScalarField   // use last component of points as scalar field (x,y,z,scalar)
  };

  // Buffer update strategy
  enum class BufferUpdateStrategy {
    kAuto,           // Automatically choose based on point count and size
    kBufferSubData,  // Always use glBufferSubData
    kMapBuffer       // Always use glMapBufferRange
  };

  // Point data update methods
  void SetPoints(const std::vector<glm::vec4>& points, ColorMode color_mode);
  void SetPoints(std::vector<glm::vec4>&& points, ColorMode color_mode);

  // Buffer management
  void PreallocateBuffers(size_t max_points);
  void SetBufferUpdateStrategy(BufferUpdateStrategy strategy) {
    buffer_update_strategy_ = strategy;
  }
  void SetBufferUpdateThreshold(size_t threshold) {
    buffer_update_threshold_ = threshold;
  }

  // Appearance settings
  void SetPointSize(float size) { point_size_ = size; }
  void SetDefaultColor(const glm::vec3& color) { default_color_ = color; }
  void SetOpacity(float opacity) { opacity_ = opacity; }
  void SetScalarRange(float min_val, float max_val) {
    min_scalar_ = min_val;
    max_scalar_ = max_val;
  }
  void SetRenderMode(PointMode mode) { render_mode_ = mode; }

  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_ != 0; }

 private:
  // Helper methods for buffer updates
  void UpdateColors(ColorMode color_mode);
  void UpdateBufferWithSubData(uint32_t buffer, const void* data,
                             size_t size_bytes, size_t offset_bytes = 0);
  void UpdateBufferWithMapping(uint32_t buffer, const void* data,
                             size_t size_bytes, size_t offset_bytes = 0);
  bool ShouldUseBufferMapping(size_t point_count) const;

  // OpenGL resources
  uint32_t vao_ = 0;
  uint32_t position_vbo_ = 0;
  uint32_t color_vbo_ = 0;
  ShaderProgram shader_;

  // Rendering data
  std::vector<glm::vec3> points_;
  std::vector<glm::vec3> colors_;

  // Appearance settings
  float point_size_ = 3.0f;
  glm::vec3 default_color_ = glm::vec3(0.25f, 0.0f, 1.0f);
  float opacity_ = 1.0f;
  float min_scalar_ = 0.0f;
  float max_scalar_ = 1.0f;
  PointMode render_mode_ = PointMode::kPoint;

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
