/**
 * @file bounding_box.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Bounding box renderer for zones, regions, and obstacles
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_BOUNDING_BOX_HPP
#define QUICKVIZ_BOUNDING_BOX_HPP

#include <glm/glm.hpp>
#include <vector>

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"

namespace quickviz {

/**
 * @brief Renderable 3D bounding box for zones and regions
 */
class BoundingBox : public OpenGlObject {
 public:
  enum class RenderMode {
    kWireframe,    // Only edges
    kSolid,        // Solid faces
    kTransparent,  // Transparent faces with edges
  };

  BoundingBox();
  BoundingBox(const glm::vec3& min_point, const glm::vec3& max_point);
  ~BoundingBox();

  // Box configuration
  void SetBounds(const glm::vec3& min_point, const glm::vec3& max_point);
  void SetCenter(const glm::vec3& center, const glm::vec3& size);
  void SetTransform(const glm::mat4& transform);
  
  // Appearance settings
  void SetColor(const glm::vec3& color);
  void SetEdgeColor(const glm::vec3& color);
  void SetOpacity(float opacity);
  void SetEdgeWidth(float width);
  void SetRenderMode(RenderMode mode);
  
  // Visibility options
  void SetShowEdges(bool show);
  void SetShowFaces(bool show);
  void SetShowCornerPoints(bool show, float point_size = 5.0f);
  
  // OpenGlObject interface
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_edges_ != 0; }

  // Utility methods
  glm::vec3 GetCenter() const;
  glm::vec3 GetSize() const;
  glm::vec3 GetMinPoint() const { return min_point_; }
  glm::vec3 GetMaxPoint() const { return max_point_; }
  
 private:
  void GenerateBoxGeometry();
  void UpdateGpuBuffers();

  // Box data
  glm::vec3 min_point_ = glm::vec3(-1.0f, -1.0f, -1.0f);
  glm::vec3 max_point_ = glm::vec3(1.0f, 1.0f, 1.0f);
  glm::mat4 transform_ = glm::mat4(1.0f);
  
  // Appearance
  glm::vec3 face_color_ = glm::vec3(0.5f, 0.5f, 0.8f);
  glm::vec3 edge_color_ = glm::vec3(0.0f, 0.0f, 0.0f);
  float opacity_ = 0.3f;
  float edge_width_ = 2.0f;
  RenderMode render_mode_ = RenderMode::kWireframe;
  
  // Visibility
  bool show_edges_ = true;
  bool show_faces_ = false;
  bool show_corner_points_ = false;
  float corner_point_size_ = 5.0f;
  
  // Geometry data
  std::vector<glm::vec3> vertices_;
  std::vector<uint32_t> edge_indices_;
  std::vector<uint32_t> face_indices_;
  
  // OpenGL resources for edges
  uint32_t vao_edges_ = 0;
  uint32_t vbo_vertices_ = 0;
  uint32_t ebo_edges_ = 0;
  
  // OpenGL resources for faces
  uint32_t vao_faces_ = 0;
  uint32_t ebo_faces_ = 0;
  
  // Shaders
  ShaderProgram edge_shader_;
  ShaderProgram face_shader_;
  
  bool needs_update_ = true;
};

}  // namespace quickviz

#endif  // QUICKVIZ_BOUNDING_BOX_HPP