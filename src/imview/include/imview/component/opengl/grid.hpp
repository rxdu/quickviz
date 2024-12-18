/*
 * @file grid.hpp
 * @date 11/2/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_GRID_HPP
#define QUICKVIZ_GRID_HPP

#include <glm/glm.hpp>

#include <vector>

#include "imview/interface/opengl_drawable.hpp"
#include "imview/component/opengl/shader_program.hpp"

namespace quickviz {
class Grid : public OpenGLDrawable {
 public:
  Grid(float grid_size = 10.0f, float spacing = 1.0f,
       glm::vec3 color = glm::vec3(0.5f, 0.5f, 0.5f));
  ~Grid();

  void SetLineColor(const glm::vec3& color, float alpha = 0.5f);

  void InitGraphicsResources() override;
  void DeinitGraphicsResources() override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view) override;

 private:
  void GenerateGrid();

  // grid parameters
  float grid_size_;
  float spacing_;
  glm::vec3 color_;
  float alpha_ = 0.5f;

  // OpenGL related
  uint32_t vao_;
  uint32_t vbo_;
  std::vector<glm::vec3> vertices_;
  ShaderProgram shader_;
};
}  // namespace quickviz

#endif  // QUICKVIZ_GRID_HPP