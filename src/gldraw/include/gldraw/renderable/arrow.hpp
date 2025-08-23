/**
 * @file arrow.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Arrow renderer for vectors, directions, and forces
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_ARROW_HPP
#define QUICKVIZ_ARROW_HPP

#include <vector>
#include <glm/glm.hpp>

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"

namespace quickviz {

/**
 * @brief Renderable 3D arrow for vectors and directions
 */
class Arrow : public OpenGlObject {
 public:
  Arrow();
  Arrow(const glm::vec3& start, const glm::vec3& end);
  ~Arrow();

  // Arrow configuration
  void SetStartPoint(const glm::vec3& start);
  void SetEndPoint(const glm::vec3& end);
  void SetDirection(const glm::vec3& origin, const glm::vec3& direction, float length);
  
  // Appearance settings
  void SetColor(const glm::vec3& color);
  void SetShaftRadius(float radius);
  void SetHeadRadius(float radius);
  void SetHeadLengthRatio(float ratio);  // Head length as ratio of total length
  
  // Rendering options
  void SetResolution(int segments);  // Number of segments for cylinder
  void SetShowAsLine(bool as_line);  // Render as simple line with cone head
  
  // OpenGlObject interface
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_shaft_ != 0; }

  // Utility methods
  float GetLength() const;
  glm::vec3 GetDirection() const;
  
 private:
  void GenerateArrowGeometry();
  void GenerateCylinder(std::vector<glm::vec3>& vertices, 
                       std::vector<uint32_t>& indices,
                       const glm::vec3& base, const glm::vec3& top,
                       float radius, int segments);
  void GenerateCone(std::vector<glm::vec3>& vertices,
                   std::vector<uint32_t>& indices,
                   const glm::vec3& base, const glm::vec3& tip,
                   float radius, int segments);
  void UpdateGpuBuffers();

  // Arrow data
  glm::vec3 start_point_ = glm::vec3(0.0f, 0.0f, 0.0f);
  glm::vec3 end_point_ = glm::vec3(1.0f, 0.0f, 0.0f);
  
  // Appearance
  glm::vec3 color_ = glm::vec3(1.0f, 0.0f, 0.0f);
  float shaft_radius_ = 0.02f;
  float head_radius_ = 0.05f;
  float head_length_ratio_ = 0.2f;
  int segments_ = 16;
  bool show_as_line_ = false;
  
  // Geometry data
  std::vector<glm::vec3> shaft_vertices_;
  std::vector<uint32_t> shaft_indices_;
  std::vector<glm::vec3> head_vertices_;
  std::vector<uint32_t> head_indices_;
  
  // OpenGL resources
  uint32_t vao_shaft_ = 0;
  uint32_t vbo_shaft_vertices_ = 0;
  uint32_t vbo_shaft_indices_ = 0;
  uint32_t vao_head_ = 0;
  uint32_t vbo_head_vertices_ = 0;
  uint32_t vbo_head_indices_ = 0;
  
  // Shader
  ShaderProgram shader_;
  
  bool needs_update_ = true;
};

}  // namespace quickviz

#endif  // QUICKVIZ_ARROW_HPP