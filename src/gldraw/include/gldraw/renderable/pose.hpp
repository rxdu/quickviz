/**
 * @file pose.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief 6-DOF pose visualization with coordinate frame and history trail
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_POSE_HPP
#define QUICKVIZ_POSE_HPP

#include <vector>
#include <deque>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"

namespace quickviz {

/**
 * @brief 6-DOF pose visualization for robotics applications
 * 
 * Renders a pose (position + orientation) with:
 * - 3D coordinate frame showing orientation
 * - Optional history trail showing past positions
 * - Configurable axis colors, lengths, and trail properties
 * - Support for both continuous and discrete pose updates
 * 
 * Common use cases:
 * - Robot current pose and path history
 * - Goal poses and waypoints
 * - Transform tree visualization
 * - 6-DOF manipulation targets
 */
class Pose : public OpenGlObject {
public:
  enum class TrailMode {
    kNone,           // No history trail
    kLine,           // Simple line connecting positions
    kDots,           // Discrete points at each position
    kArrows,         // Small arrows showing orientation history
    kFading          // Line with fading alpha based on age
  };

  Pose();
  explicit Pose(const glm::vec3& position, const glm::quat& orientation = glm::quat(1,0,0,0));
  ~Pose();

  // Pose control
  void SetPose(const glm::vec3& position, const glm::quat& orientation);
  void SetPosition(const glm::vec3& position);
  void SetOrientation(const glm::quat& orientation);
  void UpdatePose(const glm::vec3& position, const glm::quat& orientation);

  glm::vec3 GetPosition() const { return position_; }
  glm::quat GetOrientation() const { return orientation_; }

  // Coordinate frame appearance
  void SetAxisLength(float length);
  void SetAxisColors(const glm::vec3& x_color, const glm::vec3& y_color, const glm::vec3& z_color);
  void SetAxisWidth(float width);
  void SetShowFrame(bool show);
  
  float GetAxisLength() const { return axis_length_; }
  bool GetShowFrame() const { return show_frame_; }

  // History trail control
  void SetTrailMode(TrailMode mode);
  void SetTrailLength(size_t max_points);
  void SetTrailColor(const glm::vec3& color);
  void SetTrailWidth(float width);
  void SetTrailFadeTime(float seconds);
  void ClearTrail();
  
  TrailMode GetTrailMode() const { return trail_mode_; }
  size_t GetTrailLength() const { return max_trail_points_; }
  size_t GetCurrentTrailSize() const { return trail_positions_.size(); }

  // Scale and visibility
  void SetScale(float scale);
  void SetTransparency(float alpha);
  
  float GetScale() const { return scale_; }
  float GetTransparency() const { return alpha_; }

  // OpenGlObject interface
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return frame_vao_ != 0; }

private:
  struct TrailPoint {
    glm::vec3 position;
    glm::quat orientation;  // For arrow mode
    float timestamp;        // For fading mode
  };

  void GenerateFrameGeometry();
  void GenerateTrailGeometry(); 
  void UpdateFrameBuffers();
  void UpdateTrailBuffers();
  void UpdateModelMatrix();
  void AddTrailPoint(const glm::vec3& position, const glm::quat& orientation);

  // Current pose
  glm::vec3 position_;
  glm::quat orientation_;
  glm::mat4 model_matrix_;

  // Frame appearance
  float axis_length_;
  glm::vec3 x_axis_color_;
  glm::vec3 y_axis_color_;
  glm::vec3 z_axis_color_;
  float axis_width_;
  bool show_frame_;
  float scale_;
  float alpha_;

  // Trail properties
  TrailMode trail_mode_;
  size_t max_trail_points_;
  glm::vec3 trail_color_;
  float trail_width_;
  float trail_fade_time_;
  std::deque<TrailPoint> trail_positions_;

  // OpenGL resources for coordinate frame
  uint32_t frame_vao_, frame_vbo_, frame_ebo_;
  std::vector<glm::vec3> frame_vertices_;
  std::vector<glm::vec3> frame_colors_;
  std::vector<uint32_t> frame_indices_;

  // OpenGL resources for trail
  uint32_t trail_vao_, trail_vbo_, trail_ebo_;
  std::vector<glm::vec3> trail_vertices_;
  std::vector<glm::vec3> trail_vertex_colors_;
  std::vector<uint32_t> trail_indices_;

  // Shaders
  ShaderProgram frame_shader_;
  ShaderProgram trail_shader_;

  bool needs_frame_update_;
  bool needs_trail_update_;
};

} // namespace quickviz

#endif // QUICKVIZ_POSE_HPP