/**
 * @file grid.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-05
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef COMPONENT_OPENGL_GRID_HPP
#define COMPONENT_OPENGL_GRID_HPP

#include <vector>

#include <glm/glm.hpp>

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"

namespace quickviz {
class Grid : public OpenGlObject {
 public:
  Grid(float size = 10.0f, float spacing = 1.0f,
       const glm::vec3& color = glm::vec3(0.5f, 0.5f, 0.5f));
  ~Grid();

  // public methods
  void SetSize(float size);
  void SetSpacing(float spacing);
  void SetColor(const glm::vec3& color);

  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_ != 0; }

 private:
  void GenerateGrid();

  float size_ = 10.0f;
  float spacing_ = 1.0f;
  glm::vec3 color_ = glm::vec3(0.5f, 0.5f, 0.5f);

  std::vector<glm::vec3> vertices_;
  std::vector<unsigned int> indices_;

  uint32_t vao_ = 0;
  uint32_t vbo_ = 0;
  uint32_t ebo_ = 0;

  ShaderProgram shader_;
};
}  // namespace quickviz

#endif /* COMPONENT_OPENGL_GRID_HPP */