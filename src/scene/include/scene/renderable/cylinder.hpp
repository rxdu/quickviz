/**
 * @file cylinder.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Cylinder renderer for obstacles, columns, and tubes
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_CYLINDER_HPP
#define QUICKVIZ_CYLINDER_HPP

#include <glm/glm.hpp>
#include <vector>

#include "scene/renderable/geometric_primitive.hpp"
#include "../shader_program.hpp"

namespace quickviz {

/**
 * @brief Renderable 3D cylinder for obstacles and structures
 * Now inherits unified GeometricPrimitive interface
 */
class Cylinder : public GeometricPrimitive {
 public:
  // Legacy RenderMode enum for backward compatibility
  // New code should use GeometricPrimitive::RenderMode
  enum class RenderMode {
    kWireframe = static_cast<int>(GeometricPrimitive::RenderMode::kWireframe),
    kSolid = static_cast<int>(GeometricPrimitive::RenderMode::kSolid),
    kTransparent = static_cast<int>(GeometricPrimitive::RenderMode::kTransparent),
    kOutline = static_cast<int>(GeometricPrimitive::RenderMode::kOutline)
  };

  Cylinder();
  Cylinder(const glm::vec3& base_center, const glm::vec3& top_center, float radius);
  Cylinder(const glm::vec3& center, float height, float radius);
  ~Cylinder();

  // Cylinder configuration
  void SetBaseCenter(const glm::vec3& center);
  void SetTopCenter(const glm::vec3& center);
  void SetCenterAndHeight(const glm::vec3& center, float height);
  void SetRadius(float radius);
  // SetTransform is now handled by GeometricPrimitive base class
  
  // Appearance settings (forward to base class)
  void SetColor(const glm::vec3& color) override { GeometricPrimitive::SetColor(color); }
  void SetWireframeColor(const glm::vec3& color) override { GeometricPrimitive::SetWireframeColor(color); }
  void SetOpacity(float opacity) override { GeometricPrimitive::SetOpacity(opacity); }
  void SetRenderMode(RenderMode mode);  // Legacy overload
  void SetRenderMode(GeometricPrimitive::RenderMode mode) override { GeometricPrimitive::SetRenderMode(mode); }
  
  // Quality settings
  void SetResolution(int radial_segments);
  void SetWireframeWidth(float width) override { GeometricPrimitive::SetWireframeWidth(width); }
  
  // Cap settings
  void SetShowTopCap(bool show);
  void SetShowBottomCap(bool show);
  void SetShowCaps(bool show) { SetShowTopCap(show); SetShowBottomCap(show); }
  
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
  bool IsGpuResourcesAllocated() const noexcept override { return vao_sides_ != 0; }

  // Cylinder-specific utility methods
  glm::vec3 GetBaseCenter() const { return base_center_; }
  glm::vec3 GetTopCenter() const { return top_center_; }
  float GetRadius() const { return radius_; }
  float GetHeight() const;
  glm::vec3 GetAxis() const;

  // === GPU ID-Buffer Selection System ===
  bool SupportsSelection() const override { return true; }

protected:
  // =================================================================
  // Template Method Implementation
  // =================================================================
  
  void PrepareShaders(const glm::mat4& mvp_matrix, const glm::mat4& model_matrix) override;
  void RenderSolid() override;
  void RenderWireframe() override;
  void RenderPoints() override;
  
  // Cylinder-specific rendering methods
  void RenderSpecialFeatures(const glm::mat4& mvp_matrix, const glm::mat4& model_matrix);
  
  // Override ID rendering to use cylinder's geometry directly
  void RenderIdBuffer(const glm::mat4& mvp_matrix) override;
  
private:
  void GenerateCylinderGeometry();
  void UpdateGpuBuffers();

  // Cylinder data
  glm::vec3 base_center_ = glm::vec3(0.0f, 0.0f, 0.0f);
  glm::vec3 top_center_ = glm::vec3(0.0f, 1.0f, 0.0f);
  float radius_ = 0.5f;
  glm::mat4 transform_ = glm::mat4(1.0f);
  
  // Legacy appearance support (now uses base class material system)
  glm::vec3 legacy_color_ = glm::vec3(0.7f, 0.7f, 0.9f);
  glm::vec3 legacy_wireframe_color_ = glm::vec3(0.0f, 0.0f, 0.0f);
  float legacy_opacity_ = 1.0f;
  
  // Quality settings
  int radial_segments_ = 20;
  
  // Cap settings
  bool show_top_cap_ = true;
  bool show_bottom_cap_ = true;
  
  // Geometry data
  std::vector<glm::vec3> vertices_;
  std::vector<glm::vec3> normals_;
  std::vector<uint32_t> side_indices_;
  std::vector<uint32_t> top_cap_indices_;
  std::vector<uint32_t> bottom_cap_indices_;
  std::vector<uint32_t> wireframe_indices_;
  
  // OpenGL resources for sides
  uint32_t vao_sides_ = 0;
  uint32_t vbo_vertices_ = 0;
  uint32_t vbo_normals_ = 0;
  uint32_t ebo_sides_ = 0;
  
  // OpenGL resources for caps
  uint32_t vao_caps_ = 0;
  uint32_t ebo_top_cap_ = 0;
  uint32_t ebo_bottom_cap_ = 0;
  
  // OpenGL resources for wireframe
  uint32_t vao_wireframe_ = 0;
  uint32_t ebo_wireframe_ = 0;
  
  // Specialized shaders optimized for cylinder rendering with caps
  ShaderProgram solid_shader_;
  ShaderProgram wireframe_shader_;
  
  // Internal update methods
  void UpdateTransformFromCenters();
  
  // Matrices for special features rendering (stored during PrepareShaders)
  mutable glm::mat4 stored_mvp_matrix_ = glm::mat4(1.0f);
  mutable glm::mat4 stored_model_matrix_ = glm::mat4(1.0f);
};

}  // namespace quickviz

#endif  // QUICKVIZ_CYLINDER_HPP