/**
 * @file frustum.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Frustum renderer for sensor FOV, camera views, and detection zones
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_FRUSTUM_HPP
#define QUICKVIZ_FRUSTUM_HPP

#include <vector>
#include <glm/glm.hpp>

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"

namespace quickviz {

/**
 * @brief Renderable frustum for sensor FOV, camera views, and detection volumes
 * 
 * A frustum is a truncated pyramid commonly used to represent:
 * - Camera fields of view
 * - LiDAR detection cones  
 * - Radar coverage areas
 * - Sensor visibility zones
 * - Spotlight illumination volumes
 */
class Frustum : public OpenGlObject {
public:
  enum class RenderMode {
    kSolid,        // Filled faces
    kWireframe,    // Wireframe edges only
    kTransparent,  // Semi-transparent faces
    kOutline,      // Outline edges with thicker lines
    kPoints        // Corner vertices only
  };

  Frustum();
  ~Frustum();

  // Frustum definition methods
  void SetFromPerspective(const glm::vec3& origin, const glm::vec3& direction,
                         float fov_degrees, float aspect_ratio, 
                         float near_distance, float far_distance);
  
  void SetFromOrthographic(const glm::vec3& origin, const glm::vec3& direction,
                          float width, float height,
                          float near_distance, float far_distance);
  
  void SetFromCorners(const glm::vec3& origin,
                     const glm::vec3* near_corners,  // Near plane corners (4 elements)
                     const glm::vec3* far_corners);  // Far plane corners (4 elements)
                     
  void SetFromLidarSector(const glm::vec3& origin, const glm::vec3& direction,
                         float horizontal_fov, float vertical_fov,
                         float min_range, float max_range);

  // Visual properties
  void SetColor(const glm::vec3& color);
  void SetTransparency(float alpha);
  void SetRenderMode(RenderMode mode);
  void SetWireframeColor(const glm::vec3& color);
  void SetWireframeWidth(float width);
  
  // Face visibility control
  void SetShowNearFace(bool show);
  void SetShowFarFace(bool show);
  void SetShowSideFaces(bool show);
  
  // Visualization aids
  void SetShowCenterLine(bool show);
  void SetCenterLineColor(const glm::vec3& color);
  void SetShowCornerMarkers(bool show);
  void SetCornerMarkerSize(float size);

  // OpenGlObject interface
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_ != 0; }

  // Utility methods
  glm::vec3 GetOrigin() const { return origin_; }
  glm::vec3 GetDirection() const { return direction_; }
  float GetNearDistance() const { return near_distance_; }
  float GetFarDistance() const { return far_distance_; }
  
  // Intersection testing
  bool ContainsPoint(const glm::vec3& point) const;
  std::vector<glm::vec3> GetCornerPoints() const;

private:
  void GenerateFrustumGeometry();
  void UpdateGpuBuffers();
  
  // Frustum definition
  glm::vec3 origin_;
  glm::vec3 direction_;
  float near_distance_;
  float far_distance_;
  glm::vec3 near_corners_[4];  // Top-left, top-right, bottom-right, bottom-left
  glm::vec3 far_corners_[4];   // Same order as near corners
  
  // Visual properties
  glm::vec3 color_;
  float alpha_;
  RenderMode render_mode_;
  glm::vec3 wireframe_color_;
  float wireframe_width_;
  
  // Face visibility
  bool show_near_face_;
  bool show_far_face_;
  bool show_side_faces_;
  
  // Visualization aids
  bool show_center_line_;
  glm::vec3 center_line_color_;
  bool show_corner_markers_;
  float corner_marker_size_;
  
  // OpenGL resources
  unsigned int vao_, vbo_vertices_, vbo_normals_, ebo_;
  unsigned int vao_wireframe_, vbo_wireframe_, ebo_wireframe_;
  unsigned int vao_lines_, vbo_lines_;  // For center line and edges
  ShaderProgram shader_;
  ShaderProgram wireframe_shader_;
  ShaderProgram line_shader_;
  
  // Geometry data
  std::vector<glm::vec3> vertices_;
  std::vector<glm::vec3> normals_;
  std::vector<uint32_t> indices_;
  std::vector<glm::vec3> wireframe_vertices_;
  std::vector<uint32_t> wireframe_indices_;
  std::vector<glm::vec3> line_vertices_;  // Center line + corner markers
  
  bool needs_update_;
};

} // namespace quickviz

#endif // QUICKVIZ_FRUSTUM_HPP