/*
 * @file triangle.hpp
 * @date 11/4/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_TRIANGLE_HPP
#define QUICKVIZ_TRIANGLE_HPP

#include <glm/glm.hpp>

#include <vector>

#include "imview/interface/opengl_drawable.hpp"
#include "imview/component/opengl/shader_program.hpp"

namespace quickviz {
class Triangle : public OpenGLDrawable {
 public:
  Triangle(float size = 1.0f, glm::vec3 color = glm::vec3(0.5f, 0.5f, 0.5f));
  ~Triangle();

  void SetColor(const glm::vec3& color, float alpha = 0.5f);

  void OnDraw(const glm::mat4& projection, const glm::mat4& view) override;

 private:
  void GenerateTriangle();

  // triangle parameters
  float size_;
  glm::vec3 color_;
  float alpha_ = 0.5f;

  // OpenGL related
  uint32_t vao_;
  uint32_t vbo_;
  std::vector<glm::vec3> vertices_;
  ShaderProgram shader_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_TRIANGLE_HPP