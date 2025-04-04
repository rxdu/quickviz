/**
 * @file triangle.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-05
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef COMPONENT_OPENGL_TRIANGLE_HPP
#define COMPONENT_OPENGL_TRIANGLE_HPP

#include <glm/glm.hpp>

#include "renderer/interface/opengl_object.hpp"
#include "renderer/shader_program.hpp"

namespace quickviz {
class Triangle : public OpenGlObject {
 public:
  Triangle(float size = 1.0f, const glm::vec3& color = glm::vec3(1.0f, 0.5f, 0.2f));
  ~Triangle();

  // public methods
  void SetSize(float size);
  void SetColor(const glm::vec3& color);

  void AllocateGpuResources() override;
  void ReleaseGpuResources() override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;

 private:
  float size_ = 1.0f;
  glm::vec3 color_ = glm::vec3(1.0f, 0.5f, 0.2f);

  uint32_t vao_ = 0;
  uint32_t vbo_ = 0;

  ShaderProgram shader_;
};
}  // namespace quickviz

#endif /* COMPONENT_OPENGL_TRIANGLE_HPP */