/**
 * @file sphere.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Sphere renderer for waypoints, ranges, and detection zones
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_SPHERE_HPP
#define QUICKVIZ_SPHERE_HPP

#include <glm/glm.hpp>
#include <vector>

#include "gldraw/renderable/geometric_primitive.hpp"
#include "gldraw/shader_program.hpp"

namespace quickviz {

/**
 * @brief Renderable 3D sphere for waypoints and detection zones
 * Now inherits unified GeometricPrimitive interface
 */
class Sphere : public GeometricPrimitive {
 public:
  // Legacy RenderMode enum for backward compatibility
  // New code should use GeometricPrimitive::RenderMode
  enum class RenderMode {
    kWireframe = static_cast<int>(GeometricPrimitive::RenderMode::kWireframe),
    kSolid = static_cast<int>(GeometricPrimitive::RenderMode::kSolid),
    kTransparent = static_cast<int>(GeometricPrimitive::RenderMode::kTransparent),
    kPoints = static_cast<int>(GeometricPrimitive::RenderMode::kPoints)
  };

  Sphere();
  Sphere(const glm::vec3& center, float radius);
  ~Sphere();

  // Sphere configuration
  void SetCenter(const glm::vec3& center);
  void SetRadius(float radius);
  // SetTransform is now handled by GeometricPrimitive base class
  
  // Appearance settings (forward to base class)
  void SetColor(const glm::vec3& color) override { GeometricPrimitive::SetColor(color); }
  void SetWireframeColor(const glm::vec3& color) override { GeometricPrimitive::SetWireframeColor(color); }
  void SetOpacity(float opacity) override { GeometricPrimitive::SetOpacity(opacity); }
  void SetRenderMode(RenderMode mode);  // Legacy overload
  void SetRenderMode(GeometricPrimitive::RenderMode mode) override { GeometricPrimitive::SetRenderMode(mode); }
  
  // Quality settings
  void SetResolution(int latitude_segments, int longitude_segments);
  void SetWireframeWidth(float width) override { GeometricPrimitive::SetWireframeWidth(width); }
  
  // Special features
  void SetShowPoles(bool show, float pole_size = 5.0f);
  void SetShowEquator(bool show, const glm::vec3& color = glm::vec3(1.0f, 1.0f, 0.0f));
  
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
  bool IsGpuResourcesAllocated() const noexcept override { return vao_solid_ != 0; }

  // Sphere-specific utility methods
  glm::vec3 GetCenter() const { return center_; }
  float GetRadius() const { return radius_; }

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
  
  // Private special rendering methods for sphere-specific features
  void RenderSpecialFeatures(const glm::mat4& mvp_matrix, const glm::mat4& model_matrix);
  
 private:
  void GenerateSphereGeometry();
  void UpdateGpuBuffers();

  // Sphere data
  glm::vec3 center_ = glm::vec3(0.0f, 0.0f, 0.0f);
  float radius_ = 1.0f;
  glm::mat4 transform_ = glm::mat4(1.0f);
  
  // Legacy appearance support (now uses base class material system)
  glm::vec3 legacy_color_ = glm::vec3(0.7f, 0.7f, 0.9f);
  glm::vec3 legacy_wireframe_color_ = glm::vec3(0.0f, 0.0f, 0.0f);
  float legacy_opacity_ = 1.0f;
  
  // Quality settings
  int latitude_segments_ = 20;
  int longitude_segments_ = 20;
  
  // Special features
  bool show_poles_ = false;
  float pole_size_ = 5.0f;
  bool show_equator_ = false;
  glm::vec3 equator_color_ = glm::vec3(1.0f, 1.0f, 0.0f);
  
  // Geometry data
  std::vector<glm::vec3> vertices_;
  std::vector<glm::vec3> normals_;
  std::vector<uint32_t> solid_indices_;
  std::vector<uint32_t> wireframe_indices_;
  std::vector<glm::vec3> equator_vertices_;
  
  // OpenGL resources for solid rendering
  uint32_t vao_solid_ = 0;
  uint32_t vbo_vertices_ = 0;
  uint32_t vbo_normals_ = 0;
  uint32_t ebo_solid_ = 0;
  
  // OpenGL resources for wireframe rendering
  uint32_t vao_wireframe_ = 0;
  uint32_t ebo_wireframe_ = 0;
  
  // OpenGL resources for equator
  uint32_t vao_equator_ = 0;
  uint32_t vbo_equator_ = 0;
  
  // Specialized shaders optimized for parametric sphere rendering
  ShaderProgram solid_shader_;
  ShaderProgram wireframe_shader_;
  ShaderProgram id_shader_;  // Flat color shader for ID rendering
  
  // Internal update methods
  void UpdateTransformFromCenterRadius();
  
  // Matrices for special features rendering (stored during PrepareShaders)
  mutable glm::mat4 stored_mvp_matrix_ = glm::mat4(1.0f);
  mutable glm::mat4 stored_model_matrix_ = glm::mat4(1.0f);
};

}  // namespace quickviz

#endif  // QUICKVIZ_SPHERE_HPP