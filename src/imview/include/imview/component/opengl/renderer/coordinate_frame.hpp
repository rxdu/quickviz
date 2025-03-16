/**
 * @file coordinate_frame.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-16
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef OPENGL_RENDERER_COORDINATE_FRAME_HPP
#define OPENGL_RENDERER_COORDINATE_FRAME_HPP

#include <vector>
#include <glm/glm.hpp>

#include "imview/interface/opengl_object.hpp"
#include "imview/component/opengl/shader_program.hpp"

namespace quickviz {

class CoordinateFrame : public OpenGlObject {
 public:
  CoordinateFrame(float axis_length = 1.0f);
  ~CoordinateFrame();

  void SetAxisLength(float length);
  float GetAxisLength() const { return axis_length_; }
  
  // Kept for API compatibility but does nothing
  void SetShowLabels(bool show);
  bool GetShowLabels() const { return false; }

  void AllocateGpuResources() override;
  void ReleaseGpuResources() override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view) override;

 private:
  void GenerateAxes();

  float axis_length_ = 1.0f;
  
  // OpenGL related
  uint32_t vao_ = 0;
  uint32_t vbo_ = 0;
  uint32_t ebo_ = 0;
  
  std::vector<glm::vec3> vertices_;
  std::vector<glm::vec3> colors_;
  std::vector<unsigned int> indices_;
  
  ShaderProgram shader_;
};

}  // namespace quickviz

#endif /* OPENGL_RENDERER_COORDINATE_FRAME_HPP */
