/**
 * @file canvas.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-16
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef OPENGL_RENDERER_CANVAS_HPP
#define OPENGL_RENDERER_CANVAS_HPP

#include <vector>

#include <glm/glm.hpp>

#include "imview/interface/opengl_object.hpp"
#include "imview/component/opengl/shader_program.hpp"

namespace quickviz {

// Forward declaration of Point struct
struct Point;

class Canvas : public OpenGlObject {
 public:
  Canvas(float width, float height);
  ~Canvas();

  // public methods
  enum class LineType { SOLID, DASHED, DOTTED };

  void AddPoint(float x, float y, const glm::vec4& color,
                float thickness = 1.0f);
  void AddLine(float x1, float y1, float x2, float y2, const glm::vec4& color,
               float thickness = 1.0f, LineType line_type = LineType::SOLID);
  void AddRectangle(float x, float y, float width, float height,
                    const glm::vec4& color, bool filled = true,
                    float thickness = 1.0f,
                    LineType line_type = LineType::SOLID);
  void AddCircle(float x, float y, float radius, const glm::vec4& color,
                 bool filled = true, float thickness = 1.0f,
                 LineType line_type = LineType::SOLID);
  void AddEllipse(float x, float y, float rx, float ry, float angle,
                  float start_angle, float end_angle, const glm::vec4& color,
                  bool filled = true, float thickness = 1.0f,
                  LineType line_type = LineType::SOLID);
  void AddPolygon(const std::vector<glm::vec2>& points, const glm::vec4& color,
                  bool filled = true, float thickness = 1.0f,
                  LineType line_type = LineType::SOLID);
                  
  // Clear all points from the canvas
  void Clear();

  void AllocateGpuResources() override;
  void ReleaseGpuResources() override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view) override;

 private:
  ShaderProgram shader_;
  
  // Canvas dimensions
  float width_ = 0.0f;
  float height_ = 0.0f;
  
  // OpenGL objects
  unsigned int vao_ = 0;
  unsigned int vbo_ = 0;
  
  // Data storage
  std::vector<Point> points_;
};
}  // namespace quickviz

#endif /* OPENGL_RENDERER_CANVAS_HPP */
