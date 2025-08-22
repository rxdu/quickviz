/*
 * @file mock_data_generator.hpp
 * @date 2025-01-22
 * @brief Mock data generators for testing visualization contracts
 *
 * Provides sample data for testing visualization features without
 * requiring external processing libraries.
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef VISUALIZATION_MOCK_DATA_GENERATOR_HPP
#define VISUALIZATION_MOCK_DATA_GENERATOR_HPP

#include "visualization/contracts/selection_data.hpp"
#include "visualization/contracts/surface_data.hpp"
#include <random>
#include <set>
#include <numeric>
#include <algorithm>

namespace quickviz {
namespace visualization {
namespace testing {

/**
 * @brief Generates mock data for testing visualization features
 */
class MockDataGenerator {
public:
  /**
   * @brief Generate random point selection data
   * @param total_points Total number of points in the cloud
   * @param selection_ratio Fraction of points to select (0.0 to 1.0)
   * @return SelectionData with random selection
   */
  static SelectionData GenerateRandomSelection(
      size_t total_points, 
      float selection_ratio = 0.1f) {
    
    SelectionData data;
    data.selection_name = "random_selection";
    data.description = "Randomly generated selection for testing";
    
    // Generate random indices
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dis(0, total_points - 1);
    
    size_t num_selected = static_cast<size_t>(total_points * selection_ratio);
    std::set<size_t> unique_indices;
    
    while (unique_indices.size() < num_selected) {
      unique_indices.insert(dis(gen));
    }
    
    data.point_indices = std::vector<size_t>(unique_indices.begin(), unique_indices.end());
    data.highlight_color = glm::vec3(1.0f, 1.0f, 0.0f);  // Yellow
    data.size_multiplier = 2.0f;
    data.show_bounding_box = true;
    
    return data;
  }
  
  /**
   * @brief Generate rectangular selection pattern
   * @param total_points Total number of points in the cloud
   * @param start_idx Starting index for selection
   * @param count Number of consecutive points to select
   * @return SelectionData with rectangular pattern
   */
  static SelectionData GenerateRectangularSelection(
      size_t total_points,
      size_t start_idx = 0,
      size_t count = 100) {
    
    SelectionData data;
    data.selection_name = "rectangular_selection";
    data.description = "Rectangular selection pattern for testing";
    
    // Generate consecutive indices
    size_t end_idx = std::min(start_idx + count, total_points);
    data.point_indices.resize(end_idx - start_idx);
    std::iota(data.point_indices.begin(), data.point_indices.end(), start_idx);
    
    data.highlight_color = glm::vec3(0.0f, 1.0f, 0.0f);  // Green
    data.size_multiplier = 1.8f;
    data.show_bounding_box = true;
    data.show_centroid = true;
    
    return data;
  }
  
  /**
   * @brief Generate simple planar surface data
   * @param center Center point of the plane
   * @param normal Normal vector of the plane
   * @param size Size of the plane
   * @return SurfaceData for a simple plane
   */
  static SurfaceData GeneratePlanarSurface(
      const glm::vec3& center = glm::vec3(0.0f),
      const glm::vec3& normal = glm::vec3(0.0f, 0.0f, 1.0f),
      float size = 2.0f) {
    
    SurfaceData data;
    data.surface_name = "test_plane";
    data.algorithm_used = "mock_generator";
    
    // Create a simple quad
    glm::vec3 right = glm::normalize(glm::cross(normal, glm::vec3(0.0f, 1.0f, 0.0f)));
    glm::vec3 up = glm::cross(right, normal);
    
    float half_size = size * 0.5f;
    data.vertices = {
      center - right * half_size - up * half_size,  // Bottom-left
      center + right * half_size - up * half_size,  // Bottom-right
      center + right * half_size + up * half_size,  // Top-right
      center - right * half_size + up * half_size   // Top-left
    };
    
    // Two triangles to form quad
    data.triangle_indices = {0, 1, 2, 0, 2, 3};
    
    // Normals (same for all vertices of a plane)
    data.normals = {normal, normal, normal, normal};
    
    data.color = glm::vec3(0.8f, 0.6f, 0.9f);  // Light purple
    data.transparency = 0.7f;
    data.show_normals = true;
    data.normal_scale = 0.2f;
    
    return data;
  }
  
  /**
   * @brief Generate multiple overlapping selections for testing
   * @param total_points Total number of points
   * @param num_selections Number of selection regions to create
   * @return Vector of SelectionData with different colors
   */
  static std::vector<SelectionData> GenerateMultipleSelections(
      size_t total_points,
      int num_selections = 3) {
    
    std::vector<SelectionData> selections;
    std::vector<glm::vec3> colors = {
      {1.0f, 0.0f, 0.0f},  // Red
      {0.0f, 1.0f, 0.0f},  // Green  
      {0.0f, 0.0f, 1.0f},  // Blue
      {1.0f, 1.0f, 0.0f},  // Yellow
      {1.0f, 0.0f, 1.0f}   // Magenta
    };
    
    for (int i = 0; i < num_selections; ++i) {
      auto selection = GenerateRandomSelection(total_points, 0.05f);
      selection.selection_name = "selection_" + std::to_string(i);
      selection.highlight_color = colors[i % colors.size()];
      selection.size_multiplier = 1.5f + i * 0.2f;
      selections.push_back(selection);
    }
    
    return selections;
  }
};

} // namespace testing
} // namespace visualization
} // namespace quickviz

#endif // VISUALIZATION_MOCK_DATA_GENERATOR_HPP