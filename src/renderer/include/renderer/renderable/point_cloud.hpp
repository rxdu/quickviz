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
#include "renderer/renderable/layer_manager.hpp"

namespace quickviz {
class PointCloud : public OpenGlObject {
 public:
  PointCloud();
  ~PointCloud();

  // Data management
  enum class ColorMode {
    kStatic,       // use default color
    kHeightField,  // use z-coordinate of points as height field
    kScalarField,  // use last component of points as scalar field (x,y,z,scalar)
    kRGB           // use provided RGB colors per point
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
  
  // RGB point cloud support
  void SetPoints(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& colors);
  void SetPoints(std::vector<glm::vec3>&& points, std::vector<glm::vec3>&& colors);

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

  // Layer management
  LayerManager& GetLayerManager() { return layer_manager_; }
  const LayerManager& GetLayerManager() const { return layer_manager_; }
  
  std::shared_ptr<PointLayer> CreateLayer(const std::string& name, int priority = 0);
  std::shared_ptr<PointLayer> GetLayer(const std::string& name);
  bool RemoveLayer(const std::string& name);
  void ClearAllLayers();
  
  // Point highlighting
  void HighlightPoints(const std::vector<size_t>& point_indices, 
                      const glm::vec3& color,
                      const std::string& layer_name = "highlight",
                      float size_multiplier = 1.5f);
  void HighlightPoint(size_t point_index, 
                     const glm::vec3& color,
                     const std::string& layer_name = "highlight",
                     float size_multiplier = 1.5f);
  void ClearHighlights(const std::string& layer_name = "highlight");
  
  // Point selection support  
  void SetSelectedPoints(const std::vector<size_t>& point_indices, 
                        const glm::vec3& selection_color = glm::vec3(1.0f, 1.0f, 0.0f));
  void AddToSelection(const std::vector<size_t>& point_indices);
  void RemoveFromSelection(const std::vector<size_t>& point_indices);
  void ClearSelection();
  const std::vector<size_t>& GetSelectedPoints() const { return selected_points_; }
  
  // Data access for selection and PCL bridge
  size_t GetPointCount() const { return points_.size(); }
  const std::vector<glm::vec3>& GetPoints() const { return points_; }
  const std::vector<glm::vec3>& GetColors() const { return colors_; }
  
  // Convert 3D points to 4D for PCL bridge compatibility
  std::vector<glm::vec4> GetPointsAs4D() const;

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
  
  // Layer management
  LayerManager layer_manager_;
  std::vector<size_t> selected_points_;
  
  // Layer rendering support
  void UpdateLayerRendering();
  void ApplyLayerEffects(const glm::mat4& projection, const glm::mat4& view, 
                        const glm::mat4& coord_transform);
};
}  // namespace quickviz

#endif /* COMPONENT_OPENGL_POINT_CLOUD_HPP */
