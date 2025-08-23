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

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"

namespace quickviz {

/**
 * @brief Renderable 3D cylinder for obstacles and structures
 */
class Cylinder : public OpenGlObject {
 public:
  enum class RenderMode {
    kWireframe,    // Only circular edges and vertical lines
    kSolid,        // Solid filled cylinder
    kTransparent,  // Transparent filled cylinder
    kOutline       // Only top and bottom circles
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
  void SetTransform(const glm::mat4& transform);
  
  // Appearance settings
  void SetColor(const glm::vec3& color);
  void SetWireframeColor(const glm::vec3& color);
  void SetOpacity(float opacity);
  void SetRenderMode(RenderMode mode);
  
  // Quality settings
  void SetResolution(int radial_segments);
  void SetWireframeWidth(float width);
  
  // Cap settings
  void SetShowTopCap(bool show);
  void SetShowBottomCap(bool show);
  void SetShowCaps(bool show) { SetShowTopCap(show); SetShowBottomCap(show); }
  
  // OpenGlObject interface
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_sides_ != 0; }

  // Utility methods
  glm::vec3 GetBaseCenter() const { return base_center_; }
  glm::vec3 GetTopCenter() const { return top_center_; }
  float GetRadius() const { return radius_; }
  float GetHeight() const;
  glm::vec3 GetAxis() const;
  
 private:
  void GenerateCylinderGeometry();
  void UpdateGpuBuffers();

  // Cylinder data
  glm::vec3 base_center_ = glm::vec3(0.0f, 0.0f, 0.0f);
  glm::vec3 top_center_ = glm::vec3(0.0f, 1.0f, 0.0f);
  float radius_ = 0.5f;
  glm::mat4 transform_ = glm::mat4(1.0f);
  
  // Appearance
  glm::vec3 color_ = glm::vec3(0.7f, 0.7f, 0.9f);
  glm::vec3 wireframe_color_ = glm::vec3(0.0f, 0.0f, 0.0f);
  float opacity_ = 1.0f;
  RenderMode render_mode_ = RenderMode::kSolid;
  
  // Quality
  int radial_segments_ = 20;
  float wireframe_width_ = 1.0f;
  
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
  
  // Shaders
  ShaderProgram solid_shader_;
  ShaderProgram wireframe_shader_;
  
  bool needs_update_ = true;
};

}  // namespace quickviz

#endif  // QUICKVIZ_CYLINDER_HPP