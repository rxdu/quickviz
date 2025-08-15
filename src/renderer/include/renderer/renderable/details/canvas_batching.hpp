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
  std::vector<uint32_t> sequence_numbers; // Global sequence order
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
  std::vector<uint32_t> sequence_numbers; // Global sequence order per primitive
  
  // Track index ranges for each individual shape to enable individual drawing
  struct IndexRange {
    uint32_t start;  // Starting index in the indices array
    uint32_t count;  // Number of indices for this shape
  };
  std::vector<IndexRange> index_ranges; // One range per shape
  
  uint32_t vao = 0;
  uint32_t vertex_vbo = 0;
  uint32_t color_vbo = 0;
  uint32_t ebo = 0;
  bool needs_update = true;
};

/**
 * @brief Unified rendering order tracker
 * 
 * Maintains a single global sequence across all primitive types 
 * to preserve the exact order of draw calls in batching mode.
 */
struct BatchOrderTracker {
  struct OrderedPrimitive {
    enum class Type {
      kLine,
      kFilledShape,
      kOutlineShape,
      kIndividualShape  // For polygons, ellipses that use IndividualRenderStrategy
    };
    
    Type type;
    LineType line_type; // Only relevant for kLine and kOutlineShape
    uint32_t sequence_number;
    uint32_t batch_index; // Index within the specific batch, or shape index for individual shapes
    
    // Additional data for individual shapes
    enum class IndividualShapeType {
      kNone,
      kEllipse,
      kPolygon
    } individual_shape_type = IndividualShapeType::kNone;
  };
  
  std::vector<OrderedPrimitive> render_order;
  uint32_t next_sequence = 0;
  
  uint32_t GetNextSequence() { return next_sequence++; }
  void Clear() { render_order.clear(); next_sequence = 0; }
};

} // namespace quickviz

#endif /* OPENGL_RENDERER_CANVAS_BATCHING_HPP */