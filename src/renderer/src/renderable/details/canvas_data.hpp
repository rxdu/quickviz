/**
 * @file canvas_data.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-03-20
 * @brief
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef OPENGL_RENDERER_CANVAS_DATA_HPP
#define OPENGL_RENDERER_CANVAS_DATA_HPP

#include <vector>

#include <glm/glm.hpp>
#include "glad/glad.h"

#include "renderer/renderable/types.hpp"

namespace quickviz {

// Forward declaration for Canvas::LineType
class Canvas;

// Point structure to store point data
struct Point {
  glm::vec3 position;
  glm::vec4 color;
  float size;
  uint32_t sequence_number = 0;  // For maintaining draw order
};

// Line structure
struct Line {
  glm::vec3 start;
  glm::vec3 end;
  glm::vec4 color;
  float thickness;
  LineType line_type;
  uint32_t sequence_number = 0;  // For maintaining draw order
};

// Rectangle structure
struct Rectangle {
  glm::vec3 position;  // bottom-left corner
  float width;
  float height;
  glm::vec4 color;
  bool filled;
  float thickness;
  LineType line_type;
  uint32_t sequence_number = 0;  // For maintaining draw order
};

// Circle structure
struct Circle {
  glm::vec3 center;
  float radius;
  glm::vec4 color;
  bool filled;
  float thickness;
  LineType line_type;
  int num_segments;  // Number of segments to approximate the circle
  uint32_t sequence_number = 0;  // For maintaining draw order
};

// Ellipse structure
struct Ellipse {
  glm::vec3 center;
  float rx;  // x radius
  float ry;  // y radius
  float angle;  // rotation angle in radians
  float start_angle;  // start angle in radians
  float end_angle;    // end angle in radians
  glm::vec4 color;
  bool filled;
  float thickness;
  LineType line_type;
  int num_segments;  // Number of segments to approximate the ellipse
  uint32_t sequence_number = 0;  // For maintaining draw order
};

// Polygon structure
struct Polygon {
  std::vector<glm::vec3> vertices;
  glm::vec4 color;
  bool filled;
  float thickness;
  LineType line_type;
  uint32_t sequence_number = 0;  // For maintaining draw order
};

struct CanvasData {
  std::vector<Point> points;
  std::vector<Line> lines;
  std::vector<Rectangle> rectangles;
  std::vector<Circle> circles;
  std::vector<Ellipse> ellipses;
  std::vector<Polygon> polygons;
  
  // Sequence counter for maintaining draw order across all primitive types
  uint32_t next_sequence_number = 0;

  void Clear() {
    points.clear();
    lines.clear();
    rectangles.clear();
    circles.clear();
    ellipses.clear();
    polygons.clear();
    next_sequence_number = 0;
  }

  void AddPoint(float x, float y, const glm::vec4& color, float thickness) {
    Point point;
    point.position = glm::vec3(x, y, 0.0f);
    point.color = color;
    point.size = thickness;
    point.sequence_number = next_sequence_number++;
    points.push_back(point);
  }

  void AddLine(float x1, float y1, float x2, float y2, const glm::vec4& color,
               float thickness, LineType line_type) {
    Line line;
    line.start = glm::vec3(x1, y1, 0.0f);
    line.end = glm::vec3(x2, y2, 0.0f);
    line.color = color;
    line.thickness = thickness;
    line.line_type = line_type;
    line.sequence_number = next_sequence_number++;
    lines.push_back(line);
  }

  void AddRectangle(float x, float y, float width, float height,
                   const glm::vec4& color, bool filled, float thickness,
                   LineType line_type) {
    Rectangle rect;
    rect.position = glm::vec3(x, y, 0.0f);
    rect.width = width;
    rect.height = height;
    rect.color = color;
    rect.filled = filled;
    rect.thickness = thickness;
    rect.line_type = line_type;
    rect.sequence_number = next_sequence_number++;
    rectangles.push_back(rect);
  }

  void AddCircle(float x, float y, float radius, const glm::vec4& color,
                bool filled, float thickness, LineType line_type) {
    Circle circle;
    circle.center = glm::vec3(x, y, 0.0f);
    circle.radius = radius;
    circle.color = color;
    circle.filled = filled;
    circle.thickness = thickness;
    circle.line_type = line_type;
    circle.num_segments = 32;  // Default number of segments
    circle.sequence_number = next_sequence_number++;
    circles.push_back(circle);
  }

  void AddEllipse(float x, float y, float rx, float ry, float angle,
                  float start_angle, float end_angle, const glm::vec4& color,
                  bool filled, float thickness, LineType line_type) {
    Ellipse ellipse;
    ellipse.center = glm::vec3(x, y, 0.0f);
    ellipse.rx = rx;
    ellipse.ry = ry;
    ellipse.angle = angle;
    ellipse.start_angle = start_angle;
    ellipse.end_angle = end_angle;
    ellipse.color = color;
    ellipse.filled = filled;
    ellipse.thickness = thickness;
    ellipse.line_type = line_type;
    ellipse.num_segments = 32;  // Default number of segments
    ellipse.sequence_number = next_sequence_number++;
    ellipses.push_back(ellipse);
  }

  void AddPolygon(const std::vector<glm::vec2>& vertices, const glm::vec4& color,
                 bool filled, float thickness, LineType line_type) {
    Polygon polygon;
    polygon.vertices.reserve(vertices.size());
    for (const auto& v : vertices) {
      polygon.vertices.push_back(glm::vec3(v, 0.0f));
    }
    polygon.color = color;
    polygon.filled = filled;
    polygon.thickness = thickness;
    polygon.line_type = line_type;
    polygon.sequence_number = next_sequence_number++;
    polygons.push_back(polygon);
  }
};
}  // namespace quickviz

#endif /* OPENGL_RENDERER_CANVAS_DATA_HPP */
