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
#include <glm/gtc/quaternion.hpp>

#include "renderer/interface/opengl_object.hpp"
#include "renderer/shader_program.hpp"

namespace quickviz {

class CoordinateFrame : public OpenGlObject {
 public:
  // Constructor with options for 2D mode and initial pose
  CoordinateFrame(float axis_length = 1.0f, bool is_2d_mode = false);
  ~CoordinateFrame();

  // Axis length control
  void SetAxisLength(float length);
  float GetAxisLength() const { return axis_length_; }
  
  // 2D/3D mode control
  void Set2DMode(bool is_2d);
  bool Is2DMode() const { return is_2d_mode_; }
  
  // Pose control
  void SetPosition(const glm::vec3& position);
  void SetOrientation(const glm::quat& orientation);
  void SetPose(const glm::vec3& position, const glm::quat& orientation);
  
  glm::vec3 GetPosition() const { return position_; }
  glm::quat GetOrientation() const { return orientation_; }
  
  // Kept for API compatibility but does nothing
  void SetShowLabels(bool show);
  bool GetShowLabels() const { return false; }

  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view, 
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_ != 0; }

 private:
  void GenerateAxes();
  void UpdateModelMatrix();

  float axis_length_ = 1.0f;
  bool is_2d_mode_ = false;
  
  // Pose information
  glm::vec3 position_ = glm::vec3(0.0f);
  glm::quat orientation_ = glm::quat(1.0f, 0.0f, 0.0f, 0.0f); // Identity quaternion
  glm::mat4 model_matrix_ = glm::mat4(1.0f);
  
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
