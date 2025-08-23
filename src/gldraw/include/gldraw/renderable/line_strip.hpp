/**
 * @file line_strip.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Line strip renderer for paths, trajectories, and boundaries
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_LINE_STRIP_HPP
#define QUICKVIZ_LINE_STRIP_HPP

#include <vector>
#include <glm/glm.hpp>

#include "gldraw/interface/opengl_object.hpp"
#include "gldraw/shader_program.hpp"
#include "gldraw/renderable/types.hpp"

namespace quickviz {

/**
 * @brief Renderable line strip for paths, trajectories, and polylines
 */
class LineStrip : public OpenGlObject {
 public:
  LineStrip();
  ~LineStrip();

  // Line data setup
  void SetPoints(const std::vector<glm::vec3>& points);
  void SetColors(const std::vector<glm::vec3>& colors);  // Per-vertex colors
  void SetColor(const glm::vec3& color);  // Uniform color
  
  // Line appearance
  void SetLineWidth(float width);
  void SetLineType(LineType type);
  void SetClosed(bool closed);  // Connect last point to first
  void SetShowPoints(bool show, float point_size = 5.0f);
  
  // Arrow options for trajectory visualization
  void SetShowArrows(bool show, float spacing = 1.0f);
  void SetArrowSize(float size);
  
  // OpenGlObject interface
  void AllocateGpuResources() override;
  void ReleaseGpuResources() noexcept override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;
  bool IsGpuResourcesAllocated() const noexcept override { return vao_ != 0; }

  // Utility methods
  size_t GetPointCount() const { return points_.size(); }
  float GetTotalLength() const;
  
 private:
  void UpdateGpuBuffers();
  void DrawLineStrip(const glm::mat4& mvp);
  void DrawPoints(const glm::mat4& mvp);
  void DrawArrows(const glm::mat4& mvp);
  void GenerateArrowGeometry();

  // Line data
  std::vector<glm::vec3> points_;
  std::vector<glm::vec3> colors_;
  
  // Line properties
  glm::vec3 uniform_color_ = glm::vec3(0.0f, 1.0f, 0.0f);
  float line_width_ = 2.0f;
  LineType line_type_ = LineType::kSolid;
  bool closed_ = false;
  bool use_per_vertex_colors_ = false;
  
  // Point visualization
  bool show_points_ = false;
  float point_size_ = 5.0f;
  
  // Arrow visualization
  bool show_arrows_ = false;
  float arrow_spacing_ = 1.0f;
  float arrow_size_ = 0.1f;
  std::vector<glm::vec3> arrow_positions_;
  std::vector<glm::vec3> arrow_directions_;
  
  // OpenGL resources
  uint32_t vao_ = 0;
  uint32_t vertex_vbo_ = 0;
  uint32_t color_vbo_ = 0;
  uint32_t arrow_vao_ = 0;
  uint32_t arrow_vbo_ = 0;
  
  // Shaders
  ShaderProgram line_shader_;
  ShaderProgram point_shader_;
  ShaderProgram arrow_shader_;
  
  bool needs_update_ = true;
  bool needs_arrow_update_ = true;
};

}  // namespace quickviz

#endif  // QUICKVIZ_LINE_STRIP_HPP