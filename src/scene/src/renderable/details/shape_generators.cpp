/**
 * @file shape_generators.cpp
 * @author Claude Code Assistant
 * @date 2025-01-04
 * @brief Implementation of shape vertex generators
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "shape_generators.hpp"
#include <cmath>

namespace quickviz {
namespace ShapeGenerators {

std::vector<float> GenerateCircleVertices(const Circle& circle) {
  return GeometryUtils::CreateCircle(circle.center, circle.radius, 
                                    circle.num_segments, circle.filled);
}

std::vector<float> GenerateEllipseVertices(const Ellipse& ellipse) {
  // Use sequence number to determine Z depth for proper layering
  float z_depth = ellipse.sequence_number * 0.001f;
  glm::vec3 center_with_depth = {ellipse.center.x, ellipse.center.y, z_depth};
  
  return GeometryUtils::CreateEllipse(center_with_depth, ellipse.rx, ellipse.ry,
                                     ellipse.angle, ellipse.start_angle, ellipse.end_angle,
                                     ellipse.num_segments, ellipse.filled);
}

std::vector<float> GeneratePolygonVertices(const Polygon& polygon) {
  std::vector<float> vertices;
  vertices.reserve(polygon.vertices.size() * 3);
  
  // Use sequence number to determine Z depth for proper layering
  float z_depth = polygon.sequence_number * 0.001f;
  
  for (const auto& vertex : polygon.vertices) {
    vertices.insert(vertices.end(), {vertex.x, vertex.y, z_depth});
  }
  
  return vertices;
}

std::vector<float> GenerateRectangleVertices(const Rectangle& rect) {
  std::vector<float> vertices;
  
  if (rect.filled) {
    // Two triangles for filled rectangle
    vertices = {
      // Triangle 1
      rect.position.x, rect.position.y, rect.position.z,
      rect.position.x + rect.width, rect.position.y, rect.position.z,
      rect.position.x + rect.width, rect.position.y + rect.height, rect.position.z,
      
      // Triangle 2  
      rect.position.x, rect.position.y, rect.position.z,
      rect.position.x + rect.width, rect.position.y + rect.height, rect.position.z,
      rect.position.x, rect.position.y + rect.height, rect.position.z
    };
  } else {
    // Line loop for outline rectangle
    vertices = {
      rect.position.x, rect.position.y, rect.position.z,
      rect.position.x + rect.width, rect.position.y, rect.position.z,
      rect.position.x + rect.width, rect.position.y + rect.height, rect.position.z,
      rect.position.x, rect.position.y + rect.height, rect.position.z
    };
  }
  
  return vertices;
}

std::vector<float> GenerateLineVertices(const Line& line) {
  return {
    line.start.x, line.start.y, line.start.z,
    line.end.x, line.end.y, line.end.z
  };
}

std::vector<float> GeneratePointVertices(const Point& point, float size) {
  // Generate a small quad centered on the point for better visibility
  float half_size = size * 0.5f;
  
  return {
    // Triangle 1
    point.position.x - half_size, point.position.y - half_size, point.position.z,
    point.position.x + half_size, point.position.y - half_size, point.position.z,
    point.position.x + half_size, point.position.y + half_size, point.position.z,
    
    // Triangle 2
    point.position.x - half_size, point.position.y - half_size, point.position.z,
    point.position.x + half_size, point.position.y + half_size, point.position.z,
    point.position.x - half_size, point.position.y + half_size, point.position.z
  };
}

namespace GeometryUtils {

std::vector<float> CreateCircle(const glm::vec3& center, float radius, 
                               int segments, bool filled) {
  std::vector<float> vertices;
  
  if (filled) {
    // Center vertex for triangle fan
    vertices.insert(vertices.end(), {center.x, center.y, center.z});
  }
  
  // Generate circumference vertices
  for (int i = 0; i <= segments; ++i) {
    float angle = 2.0f * M_PI * i / segments;
    float x = center.x + radius * std::cos(angle);
    float y = center.y + radius * std::sin(angle);
    vertices.insert(vertices.end(), {x, y, center.z});
  }
  
  return vertices;
}

std::vector<float> CreateEllipse(const glm::vec3& center, float rx, float ry,
                                float angle, float start_angle, float end_angle,
                                int segments, bool filled) {
  std::vector<float> vertices;
  
  if (filled) {
    // Center vertex for triangle fan
    vertices.insert(vertices.end(), {center.x, center.y, center.z});
  }
  
  // Generate ellipse vertices
  for (int i = 0; i <= segments; ++i) {
    float t = start_angle + (end_angle - start_angle) * i / segments;
    
    // Local ellipse coordinates
    float x_local = rx * std::cos(t);
    float y_local = ry * std::sin(t);
    
    // Apply rotation
    float x_rotated = x_local * std::cos(angle) - y_local * std::sin(angle);
    float y_rotated = x_local * std::sin(angle) + y_local * std::cos(angle);
    
    // Translate to center
    float x = center.x + x_rotated;
    float y = center.y + y_rotated;
    
    vertices.insert(vertices.end(), {x, y, center.z});
  }
  
  return vertices;
}

} // namespace GeometryUtils

} // namespace ShapeGenerators
} // namespace quickviz