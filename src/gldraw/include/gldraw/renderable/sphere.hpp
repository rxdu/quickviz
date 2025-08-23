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

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"

namespace quickviz {

/**
 * @brief Renderable 3D sphere for waypoints and detection zones
 */
class Sphere : public OpenGlObject {
 public:
  enum class RenderMode {
    kWireframe,    // Only latitude/longitude lines
    kSolid,        // Solid filled sphere
    kTransparent,  // Transparent filled sphere
    kPoints        // Point cloud representation
  };

  Sphere();
  Sphere(const glm::vec3& center, float radius);
  ~Sphere();

  // Sphere configuration
  void SetCenter(const glm::vec3& center);
  void SetRadius(float radius);
  void SetTransform(const glm::mat4& transform);
  
  // Appearance settings
  void SetColor(const glm::vec3& color);
  void SetWireframeColor(const glm::vec3& color);
  void SetOpacity(float opacity);
  void SetRenderMode(RenderMode mode);
  
  // Quality settings
  void SetResolution(int latitude_segments, int longitude_segments);
  void SetWireframeWidth(float width);
  
  // Special features
  void SetShowPoles(bool show, float pole_size = 5.0f);
  void SetShowEquator(bool show, const glm::vec3& color = glm::vec3(1.0f, 1.0f, 0.0f));
  
  // OpenGlObject interface
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_solid_ != 0; }

  // Utility methods
  glm::vec3 GetCenter() const { return center_; }
  float GetRadius() const { return radius_; }
  float GetSurfaceArea() const;
  float GetVolume() const;
  
 private:
  void GenerateSphereGeometry();
  void UpdateGpuBuffers();

  // Sphere data
  glm::vec3 center_ = glm::vec3(0.0f, 0.0f, 0.0f);
  float radius_ = 1.0f;
  glm::mat4 transform_ = glm::mat4(1.0f);
  
  // Appearance
  glm::vec3 color_ = glm::vec3(0.7f, 0.7f, 0.9f);
  glm::vec3 wireframe_color_ = glm::vec3(0.0f, 0.0f, 0.0f);
  float opacity_ = 1.0f;
  RenderMode render_mode_ = RenderMode::kSolid;
  
  // Quality
  int latitude_segments_ = 20;
  int longitude_segments_ = 20;
  float wireframe_width_ = 1.0f;
  
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
  
  // Shaders
  ShaderProgram solid_shader_;
  ShaderProgram wireframe_shader_;
  
  bool needs_update_ = true;
};

}  // namespace quickviz

#endif  // QUICKVIZ_SPHERE_HPP