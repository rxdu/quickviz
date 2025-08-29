/**
 * @file bounding_box.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Bounding box renderer for zones, regions, and obstacles
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_BOUNDING_BOX_HPP
#define QUICKVIZ_BOUNDING_BOX_HPP

#include <glm/glm.hpp>
#include <vector>

#include "gldraw/renderable/geometric_primitive.hpp"
#include "../shader_program.hpp"

namespace quickviz {

/**
 * @brief Renderable 3D bounding box for zones and regions
 * Now inherits unified GeometricPrimitive interface
 */
class BoundingBox : public GeometricPrimitive {
 public:
  // Legacy RenderMode enum for backward compatibility
  // New code should use GeometricPrimitive::RenderMode
  enum class RenderMode {
    kWireframe = static_cast<int>(GeometricPrimitive::RenderMode::kWireframe),
    kSolid = static_cast<int>(GeometricPrimitive::RenderMode::kSolid),
    kTransparent = static_cast<int>(GeometricPrimitive::RenderMode::kTransparent),
  };

  BoundingBox();
  BoundingBox(const glm::vec3& min_point, const glm::vec3& max_point);
  ~BoundingBox();

  // Box configuration
  void SetBounds(const glm::vec3& min_point, const glm::vec3& max_point);
  void SetCenter(const glm::vec3& center, const glm::vec3& size);
  // SetTransform is now handled by GeometricPrimitive base class
  
  // Appearance settings (forward to base class)
  void SetColor(const glm::vec3& color) override { GeometricPrimitive::SetColor(color); }
  void SetEdgeColor(const glm::vec3& color);  // BoundingBox-specific
  void SetOpacity(float opacity) override { GeometricPrimitive::SetOpacity(opacity); }
  void SetEdgeWidth(float width);  // BoundingBox-specific
  void SetRenderMode(RenderMode mode);  // Legacy overload
  void SetRenderMode(GeometricPrimitive::RenderMode mode) override { GeometricPrimitive::SetRenderMode(mode); }
  
  // Visibility options
  void SetShowEdges(bool show);
  void SetShowFaces(bool show);
  void SetShowCornerPoints(bool show, float point_size = 5.0f);
  
  // =================================================================
  // GeometricPrimitive Interface Implementation
  // =================================================================
  
  // Transform interface
  void SetTransform(const glm::mat4& transform) override;
  glm::mat4 GetTransform() const override;
  
  // Geometry calculations
  float GetVolume() const override;
  float GetSurfaceArea() const override;
  glm::vec3 GetCentroid() const override;
  std::pair<glm::vec3, glm::vec3> GetBoundingBox() const override;
  
  // OpenGL resource management
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_edges_ != 0; }

  // BoundingBox-specific utility methods
  glm::vec3 GetCenter() const;
  glm::vec3 GetSize() const;
  glm::vec3 GetMinPoint() const { return min_point_; }
  glm::vec3 GetMaxPoint() const { return max_point_; }

protected:
  // =================================================================
  // Template Method Implementation
  // =================================================================
  
  void PrepareShaders(const glm::mat4& mvp_matrix, const glm::mat4& model_matrix) override;
  void RenderSolid() override;
  void RenderWireframe() override;
  void RenderPoints() override;
  
  // Override ID rendering to use bounding box's geometry directly
  void RenderIdBuffer(const glm::mat4& mvp_matrix) override;
  
  // BoundingBox-specific rendering methods
  void RenderSpecialFeatures(const glm::mat4& mvp_matrix, const glm::mat4& model_matrix);
  
private:
  void GenerateBoxGeometry();
  void UpdateGpuBuffers();

  // Box data
  glm::vec3 min_point_ = glm::vec3(-1.0f, -1.0f, -1.0f);
  glm::vec3 max_point_ = glm::vec3(1.0f, 1.0f, 1.0f);
  glm::mat4 transform_ = glm::mat4(1.0f);
  
  // Legacy appearance support (now uses base class material system)
  glm::vec3 legacy_face_color_ = glm::vec3(0.5f, 0.5f, 0.8f);
  glm::vec3 edge_color_ = glm::vec3(0.0f, 0.0f, 0.0f);  // BoundingBox-specific
  float legacy_opacity_ = 0.3f;
  float edge_width_ = 2.0f;
  
  // Visibility
  bool show_edges_ = true;
  bool show_faces_ = false;
  bool show_corner_points_ = false;
  float corner_point_size_ = 5.0f;
  
  // Geometry data
  std::vector<glm::vec3> vertices_;
  std::vector<uint32_t> edge_indices_;
  std::vector<uint32_t> face_indices_;
  
  // OpenGL resources for edges
  uint32_t vao_edges_ = 0;
  uint32_t vbo_vertices_ = 0;
  uint32_t ebo_edges_ = 0;
  
  // OpenGL resources for faces
  uint32_t vao_faces_ = 0;
  uint32_t ebo_faces_ = 0;
  
  // Specialized shaders optimized for bounding box rendering
  ShaderProgram edge_shader_;
  ShaderProgram face_shader_;
  
  // Internal update methods
  void UpdateTransformFromBounds();
  
  // Matrices for special features rendering (stored during PrepareShaders)
  mutable glm::mat4 stored_mvp_matrix_ = glm::mat4(1.0f);
  mutable glm::mat4 stored_model_matrix_ = glm::mat4(1.0f);
};

}  // namespace quickviz

#endif  // QUICKVIZ_BOUNDING_BOX_HPP