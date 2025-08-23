/**
 * @file plane.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Plane renderer for extracted surfaces, ground planes, and walls
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_PLANE_HPP
#define QUICKVIZ_PLANE_HPP

#include <glm/glm.hpp>
#include <vector>

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"

namespace quickviz {

/**
 * @brief Renderable plane for surfaces, ground planes, and walls
 */
class Plane : public OpenGlObject {
 public:
  enum class RenderMode {
    kSolid,        // Solid filled plane
    kWireframe,    // Only edges/grid lines
    kTransparent,  // Transparent filled plane
    kPoints        // Point grid representation
  };

  Plane();
  Plane(const glm::vec3& center, const glm::vec3& normal, const glm::vec2& size);
  Plane(const glm::vec3& point1, const glm::vec3& point2, const glm::vec3& point3, const glm::vec3& point4);
  ~Plane();

  // Plane configuration
  void SetCenter(const glm::vec3& center);
  void SetNormal(const glm::vec3& normal);
  void SetSize(const glm::vec2& size);  // width, height
  void SetFromCorners(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3, const glm::vec3& p4);
  void SetFromPointAndNormal(const glm::vec3& point, const glm::vec3& normal, const glm::vec2& size);
  void SetTransform(const glm::mat4& transform);
  
  // Appearance settings
  void SetColor(const glm::vec3& color);
  void SetWireframeColor(const glm::vec3& color);
  void SetOpacity(float opacity);
  void SetRenderMode(RenderMode mode);
  
  // Grid settings
  void SetGridResolution(int width_segments, int height_segments);
  void SetShowGrid(bool show);
  void SetGridColor(const glm::vec3& color);
  void SetWireframeWidth(float width);
  
  // Normal visualization
  void SetShowNormal(bool show, float length = 1.0f);
  void SetNormalColor(const glm::vec3& color);
  
  // Texture support
  void SetTextureCoordinates(bool enable);
  
  // OpenGlObject interface
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_plane_ != 0; }

  // Utility methods
  glm::vec3 GetCenter() const { return center_; }
  glm::vec3 GetNormal() const { return normal_; }
  glm::vec2 GetSize() const { return size_; }
  float GetArea() const { return size_.x * size_.y; }
  glm::vec4 GetPlaneEquation() const;  // ax + by + cz + d = 0
  
 private:
  void GeneratePlaneGeometry();
  void UpdateGpuBuffers();

  // Plane data
  glm::vec3 center_ = glm::vec3(0.0f, 0.0f, 0.0f);
  glm::vec3 normal_ = glm::vec3(0.0f, 1.0f, 0.0f);
  glm::vec2 size_ = glm::vec2(2.0f, 2.0f);
  glm::mat4 transform_ = glm::mat4(1.0f);
  
  // Appearance
  glm::vec3 color_ = glm::vec3(0.7f, 0.7f, 0.9f);
  glm::vec3 wireframe_color_ = glm::vec3(0.0f, 0.0f, 0.0f);
  glm::vec3 grid_color_ = glm::vec3(0.5f, 0.5f, 0.5f);
  glm::vec3 normal_color_ = glm::vec3(0.0f, 1.0f, 0.0f);
  float opacity_ = 1.0f;
  RenderMode render_mode_ = RenderMode::kSolid;
  
  // Grid settings
  int width_segments_ = 10;
  int height_segments_ = 10;
  bool show_grid_ = false;
  float wireframe_width_ = 1.0f;
  
  // Normal visualization
  bool show_normal_ = false;
  float normal_length_ = 1.0f;
  
  // Texture settings
  bool use_texture_coords_ = false;
  
  // Geometry data
  std::vector<glm::vec3> vertices_;
  std::vector<glm::vec3> normals_;
  std::vector<glm::vec2> tex_coords_;
  std::vector<uint32_t> solid_indices_;
  std::vector<uint32_t> wireframe_indices_;
  std::vector<glm::vec3> normal_lines_;
  
  // OpenGL resources for plane
  uint32_t vao_plane_ = 0;
  uint32_t vbo_vertices_ = 0;
  uint32_t vbo_normals_ = 0;
  uint32_t vbo_tex_coords_ = 0;
  uint32_t ebo_solid_ = 0;
  
  // OpenGL resources for wireframe/grid
  uint32_t vao_wireframe_ = 0;
  uint32_t ebo_wireframe_ = 0;
  
  // OpenGL resources for normal lines
  uint32_t vao_normals_ = 0;
  uint32_t vbo_normal_lines_ = 0;
  
  // Shaders
  ShaderProgram solid_shader_;
  ShaderProgram wireframe_shader_;
  
  bool needs_update_ = true;
};

}  // namespace quickviz

#endif  // QUICKVIZ_PLANE_HPP