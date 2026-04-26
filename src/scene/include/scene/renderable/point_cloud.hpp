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
#include <unordered_map>

#include <glm/glm.hpp>

#include "scene/interface/opengl_object.hpp"
#include "../shader_program.hpp"
#include "scene/renderable/types.hpp"
#include "details/point_layer_manager.hpp"

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
  float GetPointSize() const { return point_size_; }
  void SetDefaultColor(const glm::vec3& color) { default_color_ = color; }
  void SetOpacity(float opacity) { opacity_ = opacity; }
  void SetScalarRange(float min_val, float max_val) {
    min_scalar_ = min_val;
    max_scalar_ = max_val;
  }
  void SetRenderMode(PointMode mode) { render_mode_ = mode; }
  PointMode GetRenderMode() const { return render_mode_; }
  
  // Selection support
  void SetObjectIdBase(uint32_t object_id);

  // Layer management
  PointLayerManager& GetLayerManager() { return layer_manager_; }
  const PointLayerManager& GetLayerManager() const { return layer_manager_; }
  
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
  
  
  // Data access for selection and PCL bridge
  size_t GetPointCount() const { return points_.size(); }
  const std::vector<glm::vec3>& GetPoints() const { return points_; }
  const std::vector<glm::vec3>& GetColors() const { return colors_; }
  
  // Convert 3D points to 4D for PCL bridge compatibility
  std::vector<glm::vec4> GetPointsAs4D() const;
  
  // ID buffer support for GPU picking
  static glm::vec3 EncodePointId(size_t point_index);
  static size_t DecodePointId(const glm::vec3& color);
  static size_t DecodePointId(uint8_t r, uint8_t g, uint8_t b);

  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_ != 0; }

  // === Enhanced Selection System ===
  bool SupportsSelection() const override { return true; }
  bool SupportsPointPicking() const override { return true; }
  SelectionPriority GetSelectionPriority() const override { return SelectionPriority::kPoint; }
  
  size_t PickPointAt(float screen_x, float screen_y,
                    float screen_width, float screen_height,
                    const glm::mat4& projection,
                    const glm::mat4& view,
                    const glm::mat4& coord_transform = glm::mat4(1.0f)) const override;
  
  glm::vec3 GetPointPosition(size_t point_index) const override {
    if (point_index < points_.size()) {
      return points_[point_index];
    }
    return glm::vec3(0.0f);
  }

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
  uint32_t id_vbo_ = 0;  // VBO for point indices in ID mode
  uint32_t id_vao_ = 0;  // Dedicated VAO for ID buffer rendering
  ShaderProgram shader_;
  ShaderProgram id_shader_;  // Shader for ID buffer rendering

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
  uint32_t object_id_base_ = 0;  // Base object ID for point selection

  // Buffer management
  size_t buffer_capacity_ = 0;
  size_t active_points_ = 0;
  bool buffers_preallocated_ = false;
  BufferUpdateStrategy buffer_update_strategy_ = BufferUpdateStrategy::kAuto;
  size_t buffer_update_threshold_ = 10000;  // Default threshold: 10,000 points
  bool needs_update_ = false;
  
  // Layer management
  PointLayerManager layer_manager_;
  
  // Layer rendering support
  void UpdateLayerRendering();
  void ApplyLayerEffects(const glm::mat4& projection, const glm::mat4& view, 
                        const glm::mat4& coord_transform);
  
  // Layer index buffer management
  struct LayerIndexBuffer {
    uint32_t ebo = 0;  // Element Buffer Object (index buffer)
    size_t count = 0;   // Number of indices
    bool needs_update = true;
  };
  std::unordered_map<std::string, LayerIndexBuffer> layer_index_buffers_;
  void UpdateLayerIndexBuffer(const std::string& layer_name, 
                              const std::vector<size_t>& indices);
  void InvalidateLayerBuffer(const std::string& layer_name);
  void CleanupLayerIndexBuffers();
};
}  // namespace quickviz

#endif /* COMPONENT_OPENGL_POINT_CLOUD_HPP */
