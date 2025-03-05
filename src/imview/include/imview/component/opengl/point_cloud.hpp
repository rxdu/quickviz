/**
 * @file point_cloud.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-05
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef COMPONENT_OPENGL_POINT_CLOUD_HPP
#define COMPONENT_OPENGL_POINT_CLOUD_HPP

#include <vector>
#include <memory>

#include <glm/glm.hpp>
#include "imview/interface/opengl_drawable.hpp"
#include "imview/component/opengl/shader_program.hpp"

namespace quickviz {

enum class PointRenderMode {
  Points,
  Spheres
};

class PointCloud : public OpenGLDrawable {
 public:
  PointCloud();
  ~PointCloud();

  // Data management
  void SetPoints(const std::vector<glm::vec3>& points);
  void SetColors(const std::vector<glm::vec3>& colors);
  void SetScalarField(const std::vector<float>& scalars, float min_val = 0.0f, float max_val = 1.0f);
  
  // Appearance
  void SetPointSize(float size) { point_size_ = size; }
  void SetOpacity(float opacity) { opacity_ = opacity; }
  void SetRenderMode(PointRenderMode mode) { render_mode_ = mode; }

 private:
  void InitGraphicsResources() override;
  void DeinitGraphicsResources() override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view) override;

  // OpenGL resources
  uint32_t vao_ = 0;
  uint32_t position_vbo_ = 0;
  uint32_t color_vbo_ = 0;
  ShaderProgram shader_;

  // Data
  std::vector<glm::vec3> points_;
  std::vector<glm::vec3> colors_;
  
  // Appearance
  float point_size_ = 3.0f;
  float opacity_ = 1.0f;
  PointRenderMode render_mode_ = PointRenderMode::Points;

  bool needs_update_ = false;
};
}  // namespace quickviz

#endif /* COMPONENT_OPENGL_POINT_CLOUD_HPP */
