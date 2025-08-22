/*
 * @file surface_data.hpp
 * @date 2025-01-22
 * @brief Clean data contract for surface visualization
 *
 * Data structure for external applications to describe extracted
 * surfaces/planes that should be visualized.
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef VISUALIZATION_SURFACE_DATA_HPP
#define VISUALIZATION_SURFACE_DATA_HPP

#include <vector>
#include <string>
#include <glm/glm.hpp>

namespace quickviz {
namespace visualization {

/**
 * @brief Data contract for surface visualization
 */
struct SurfaceData {
  // Surface geometry
  std::vector<glm::vec3> vertices;
  std::vector<uint32_t> triangle_indices;
  std::vector<glm::vec3> normals;  // Optional, per-vertex normals
  
  // Original point cloud references (optional)
  std::vector<size_t> source_point_indices;
  
  // Visual properties
  glm::vec3 color{0.7f, 0.7f, 0.9f};  // Default light blue
  float transparency = 0.6f;
  bool show_wireframe = false;
  glm::vec3 wireframe_color{0.0f, 0.0f, 0.0f};  // Default black
  
  // Normal visualization
  bool show_normals = false;
  float normal_scale = 0.1f;
  glm::vec3 normal_color{0.0f, 1.0f, 0.0f};  // Default green
  
  // Metadata
  std::string surface_name = "surface";
  std::string algorithm_used;
  float fitting_error = 0.0f;
  
  // Utility methods
  bool IsEmpty() const { return vertices.empty(); }
  size_t GetVertexCount() const { return vertices.size(); }
  size_t GetTriangleCount() const { return triangle_indices.size() / 3; }
  bool HasNormals() const { return normals.size() == vertices.size(); }
};

} // namespace visualization
} // namespace quickviz

#endif // VISUALIZATION_SURFACE_DATA_HPP