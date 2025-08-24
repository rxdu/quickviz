/**
 * @file vector_field.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-24
 * @brief Vector field renderer for force fields, velocity fields, and gradients
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_VECTOR_FIELD_HPP
#define QUICKVIZ_VECTOR_FIELD_HPP

#include <vector>
#include <functional>
#include <glm/glm.hpp>

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"

namespace quickviz {

/**
 * @brief Efficient vector field renderer using instanced rendering
 * 
 * This class renders multiple arrows efficiently using GPU instancing,
 * avoiding duplication with the Arrow class by using a shared arrow mesh
 * that is rendered multiple times with different transformations.
 */
class VectorField : public OpenGlObject {
 public:
  enum class ColorMode {
    kUniform,     // Single color for all vectors
    kMagnitude,   // Color based on vector magnitude
    kDirection,   // Color based on vector direction
    kCustom       // User-specified colors per vector
  };

  enum class RenderStyle {
    kArrows3D,    // Full 3D arrows with lighting
    kLines,       // Simple lines
    kFlat2D       // 2D arrows on XY plane
  };

  VectorField();
  ~VectorField();

  // Data management
  void SetVectors(const std::vector<glm::vec3>& origins,
                  const std::vector<glm::vec3>& vectors);
  void AddVector(const glm::vec3& origin, const glm::vec3& vector);
  void ClearVectors();
  size_t GetVectorCount() const { return origins_.size(); }

  // Grid generation
  void GenerateGridField(const glm::vec3& min_corner,
                        const glm::vec3& max_corner,
                        const glm::ivec3& resolution,
                        std::function<glm::vec3(const glm::vec3&)> field_function);

  // Appearance
  void SetColorMode(ColorMode mode);
  void SetUniformColor(const glm::vec3& color);
  void SetCustomColors(const std::vector<glm::vec3>& colors);
  void SetColorRange(float min_magnitude, float max_magnitude);
  
  // Rendering options
  void SetRenderStyle(RenderStyle style);
  void SetArrowScale(float scale);
  void SetLineWidth(float width);
  void SetOpacity(float opacity);
  
  // Performance options
  void SetSubsampling(float ratio);  // Show only a fraction of vectors
  void SetMagnitudeThreshold(float min_magnitude);  // Hide small vectors
  void SetLevelOfDetail(bool enabled, float distance_threshold = 10.0f);

  // Query
  glm::vec3 GetVector(size_t index) const;
  glm::vec3 GetOrigin(size_t index) const;
  float GetMaxMagnitude() const;
  void GetBoundingBox(glm::vec3& min_corner, glm::vec3& max_corner) const;

  // OpenGlObject interface
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_ != 0; }

 private:
  // Vector data
  std::vector<glm::vec3> origins_;
  std::vector<glm::vec3> vectors_;
  std::vector<glm::vec3> colors_;
  
  // Computed data for rendering
  std::vector<glm::mat4> transforms_;  // Per-instance transformation matrices
  std::vector<glm::vec3> instance_colors_;  // Per-instance colors
  std::vector<uint32_t> visible_indices_;  // Indices of visible vectors after filtering
  
  // Rendering settings
  ColorMode color_mode_ = ColorMode::kUniform;
  RenderStyle render_style_ = RenderStyle::kArrows3D;
  glm::vec3 uniform_color_ = glm::vec3(0.0f, 0.7f, 1.0f);
  float arrow_scale_ = 1.0f;
  float line_width_ = 2.0f;
  float opacity_ = 1.0f;
  
  // Performance settings
  float subsampling_ratio_ = 1.0f;
  float magnitude_threshold_ = 0.01f;
  bool use_lod_ = false;
  float lod_distance_ = 10.0f;
  
  // Color mapping
  float min_magnitude_ = 0.0f;
  float max_magnitude_ = 1.0f;
  
  // Shared arrow geometry (single arrow pointing in +Z direction)
  std::vector<glm::vec3> arrow_vertices_;
  std::vector<glm::vec3> arrow_normals_;
  std::vector<uint32_t> arrow_indices_;
  
  // Line geometry for line mode
  std::vector<glm::vec3> line_vertices_;
  std::vector<glm::vec3> line_colors_;
  
  // OpenGL resources
  uint32_t vao_ = 0;
  uint32_t vbo_vertices_ = 0;
  uint32_t vbo_normals_ = 0;
  uint32_t vbo_transforms_ = 0;  // Instance buffer for transformation matrices
  uint32_t vbo_colors_ = 0;       // Instance buffer for colors
  uint32_t ebo_ = 0;
  
  uint32_t vao_lines_ = 0;
  uint32_t vbo_line_vertices_ = 0;
  uint32_t vbo_line_colors_ = 0;
  
  // Shaders
  ShaderProgram arrow_shader_;
  ShaderProgram line_shader_;
  
  // Update flags
  bool needs_transform_update_ = true;
  bool needs_color_update_ = true;
  bool needs_visibility_update_ = true;
  
  // Helper methods
  void CreateArrowGeometry();
  void UpdateTransforms();
  void UpdateColors();
  void UpdateVisibility();
  void UpdateLineGeometry();
  
  glm::vec3 ComputeColorForVector(const glm::vec3& vector, size_t index) const;
  glm::mat4 ComputeTransformForVector(const glm::vec3& origin, const glm::vec3& vector) const;
  bool ShouldRenderVector(const glm::vec3& vector, size_t index) const;
};

}  // namespace quickviz

#endif  // QUICKVIZ_VECTOR_FIELD_HPP