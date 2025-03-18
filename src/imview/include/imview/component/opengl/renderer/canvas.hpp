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
#include <string>

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

  void AddBackgroundImage(const std::string& image_path, const glm::vec3& origin, float resolution);

  // Clear all points from the canvas
  void Clear();

  void AllocateGpuResources() override;
  void ReleaseGpuResources() override;
  void OnDraw(const glm::mat4& projection, const glm::mat4& view,
              const glm::mat4& coord_transform = glm::mat4(1.0f)) override;

 private:
  struct Point {
    glm::vec3 position;
    glm::vec4 color;
    float size;
  };

  // Load and setup background image
  void SetupBackgroundImage();

  float width_;
  float height_;
  std::vector<Point> points_;

  // Background image resources
  bool has_background_ = false;
  std::string background_image_path_;
  unsigned char* background_image_data_ = nullptr;
  int background_image_width_ = 0;
  int background_image_height_ = 0;
  int background_image_channels_ = 0;
  uint32_t background_texture_ = 0;
  uint32_t background_vao_ = 0;
  uint32_t background_vbo_ = 0;
  ShaderProgram background_shader_;

  // Primitive rendering resources
  uint32_t primitive_vao_ = 0;
  uint32_t primitive_vbo_ = 0;
  ShaderProgram primitive_shader_;
};

}  // namespace quickviz

#endif /* OPENGL_RENDERER_CANVAS_HPP */
