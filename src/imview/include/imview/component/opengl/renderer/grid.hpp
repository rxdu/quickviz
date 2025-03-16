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

#include "imview/interface/opengl_object.hpp"
#include "imview/component/opengl/shader_program.hpp"

namespace quickviz {
class Grid : public OpenGlObject {
 public:
  Grid(float grid_size = 10.0f, float spacing = 1.0f,
       glm::vec3 color = glm::vec3(0.5f, 0.5f, 0.5f));
  ~Grid();

  void SetLineColor(const glm::vec3& color, float alpha = 0.5f);

  // Accessor methods
  float GetGridSize() const { return grid_size_; }
  float GetSpacing() const { return spacing_; }
  glm::vec3 GetColor() const { return color_; }
  float GetAlpha() const { return alpha_; }

  void AllocateGpuResources() override;
  void ReleaseGpuResources() override;
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