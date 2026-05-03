/**
 * @file shape_generators.hpp
 * @author Claude Code Assistant
 * @date 2025-01-04
 * @brief Shape vertex generators for unified renderer
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef OPENGL_RENDERER_SHAPE_GENERATORS_HPP
#define OPENGL_RENDERER_SHAPE_GENERATORS_HPP

#include <vector>
#include <glm/glm.hpp>
#include "canvas_data.hpp"

namespace quickviz {

/**
 * @brief Collection of vertex generation functions for different shape types
 * 
 * These functions generate vertex data that can be used with the unified
 * ShapeRenderer, eliminating code duplication in shape-specific rendering.
 */
namespace ShapeGenerators {

/**
 * @brief Generate vertices for a circle
 * @param circle Circle data
 * @return Flat array of vertex positions
 */
std::vector<float> GenerateCircleVertices(const Circle& circle);

/**
 * @brief Generate vertices for an ellipse  
 * @param ellipse Ellipse data
 * @return Flat array of vertex positions
 */
std::vector<float> GenerateEllipseVertices(const Ellipse& ellipse);

/**
 * @brief Generate vertices for a polygon
 * @param polygon Polygon data
 * @return Flat array of vertex positions
 */
std::vector<float> GeneratePolygonVertices(const Polygon& polygon);

/**
 * @brief Generate vertices for a rectangle
 * @param rect Rectangle data
 * @return Flat array of vertex positions
 */
std::vector<float> GenerateRectangleVertices(const Rectangle& rect);

/**
 * @brief Generate vertices for a line
 * @param line Line data
 * @return Flat array of vertex positions
 */
std::vector<float> GenerateLineVertices(const Line& line);

/**
 * @brief Generate vertices for a point (as a small quad for visibility)
 * @param point Point data
 * @param size Point size for quad generation
 * @return Flat array of vertex positions
 */
std::vector<float> GeneratePointVertices(const Point& point, float size = 0.01f);

/**
 * @brief Helper functions for common geometry operations
 */
namespace GeometryUtils {
  /**
   * @brief Generate vertices for a circle with specified parameters
   * @param center Circle center
   * @param radius Circle radius
   * @param segments Number of segments for tessellation
   * @param filled Whether to include center vertex for filled circles
   * @return Flat array of vertex positions
   */
  std::vector<float> CreateCircle(const glm::vec3& center, float radius, 
                                 int segments, bool filled);
  
  /**
   * @brief Generate vertices for an ellipse with specified parameters
   * @param center Ellipse center
   * @param rx X-axis radius
   * @param ry Y-axis radius
   * @param angle Rotation angle in radians
   * @param start_angle Start angle for partial ellipses
   * @param end_angle End angle for partial ellipses
   * @param segments Number of segments for tessellation
   * @param filled Whether to include center vertex
   * @return Flat array of vertex positions
   */
  std::vector<float> CreateEllipse(const glm::vec3& center, float rx, float ry,
                                  float angle, float start_angle, float end_angle,
                                  int segments, bool filled);
}

} // namespace ShapeGenerators

} // namespace quickviz

#endif /* OPENGL_RENDERER_SHAPE_GENERATORS_HPP */