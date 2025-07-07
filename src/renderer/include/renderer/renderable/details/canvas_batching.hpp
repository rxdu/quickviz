/**
 * @file canvas_batching.hpp
 * @author Claude Code Assistant
 * @date 2025-01-04
 * @brief Batching structures for Canvas rendering optimization
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef OPENGL_RENDERER_CANVAS_BATCHING_HPP
#define OPENGL_RENDERER_CANVAS_BATCHING_HPP

#include <vector>
#include <cstdint>
#include <glm/glm.hpp>
#include "renderer/renderable/types.hpp"

namespace quickviz {

/**
 * @brief Batched line rendering data structure
 * 
 * Aggregates multiple lines into a single draw call for better performance.
 */
struct LineBatch {
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec4> colors;
  std::vector<float> thicknesses;
  std::vector<LineType> line_types;
  uint32_t vao = 0;
  uint32_t position_vbo = 0;
  uint32_t color_vbo = 0;
  bool needs_update = true;
};

/**
 * @brief Batched shape rendering data structure
 * 
 * Aggregates multiple shapes (rectangles, circles) into batches
 * for efficient rendering with reduced draw calls.
 */
struct ShapeBatch {
  std::vector<float> vertices;
  std::vector<uint32_t> indices;
  std::vector<glm::vec4> colors;
  uint32_t vao = 0;
  uint32_t vertex_vbo = 0;
  uint32_t color_vbo = 0;
  uint32_t ebo = 0;
  bool needs_update = true;
};

} // namespace quickviz

#endif /* OPENGL_RENDERER_CANVAS_BATCHING_HPP */