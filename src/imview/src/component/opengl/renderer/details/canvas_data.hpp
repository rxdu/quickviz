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

namespace quickviz {
// Point structure to store point data
struct Point {
  glm::vec3 position;
  glm::vec4 color;
  float size;
};

struct CanvasData {
  std::vector<Point> points;

  void Clear() { points.clear(); }
};
}  // namespace quickviz

#endif /* OPENGL_RENDERER_CANVAS_DATA_HPP */
