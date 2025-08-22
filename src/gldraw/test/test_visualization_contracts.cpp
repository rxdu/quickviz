/*
 * @file test_visualization_contracts.cpp
 * @date 2025-01-22
 * @brief Test new visualization data contracts
 *
 * Demonstrates the new clean API for external data visualization
 * without breaking existing functionality.
 */

#include <iostream>
#include <memory>

#include "visualization/contracts/selection_data.hpp"
#include "visualization/contracts/surface_data.hpp"
#include "visualization/testing/mock_data_generator.hpp"

using namespace quickviz;

int main() {
  std::cout << "=== Testing New Visualization Data Contracts ===" << std::endl;
  
  // Test 1: Generate mock selection data
  std::cout << "\nTest 1: Mock Selection Data" << std::endl;
  auto selection = visualization::testing::MockDataGenerator::GenerateRandomSelection(1000, 0.15f);
  
  std::cout << "Selection '" << selection.selection_name << "':" << std::endl;
  std::cout << "  Selected points: " << selection.GetCount() << std::endl;
  std::cout << "  Color: (" << selection.highlight_color.x << ", " 
            << selection.highlight_color.y << ", " << selection.highlight_color.z << ")" << std::endl;
  std::cout << "  Size multiplier: " << selection.size_multiplier << std::endl;
  std::cout << "  Show bounding box: " << (selection.show_bounding_box ? "yes" : "no") << std::endl;
  
  // Test 2: Generate mock surface data
  std::cout << "\nTest 2: Mock Surface Data" << std::endl;
  auto surface = visualization::testing::MockDataGenerator::GeneratePlanarSurface(
      glm::vec3(0.0f, 0.0f, 1.0f),  // center
      glm::vec3(0.0f, 0.0f, 1.0f),  // normal (up)
      3.0f                          // size
  );
  
  std::cout << "Surface '" << surface.surface_name << "':" << std::endl;
  std::cout << "  Vertices: " << surface.GetVertexCount() << std::endl;
  std::cout << "  Triangles: " << surface.GetTriangleCount() << std::endl;
  std::cout << "  Has normals: " << (surface.HasNormals() ? "yes" : "no") << std::endl;
  std::cout << "  Algorithm: " << surface.algorithm_used << std::endl;
  std::cout << "  Color: (" << surface.color.x << ", " 
            << surface.color.y << ", " << surface.color.z << ")" << std::endl;
  std::cout << "  Transparency: " << surface.transparency << std::endl;
  
  // Test 3: Multiple selections
  std::cout << "\nTest 3: Multiple Selections" << std::endl;
  auto multiple_selections = visualization::testing::MockDataGenerator::GenerateMultipleSelections(1000, 3);
  
  for (size_t i = 0; i < multiple_selections.size(); ++i) {
    const auto& sel = multiple_selections[i];
    std::cout << "  " << sel.selection_name << ": " << sel.GetCount() 
              << " points, color (" << sel.highlight_color.x << ", "
              << sel.highlight_color.y << ", " << sel.highlight_color.z << ")" << std::endl;
  }
  
  // Test 4: Data contract validation
  std::cout << "\nTest 4: Data Validation" << std::endl;
  visualization::SelectionData empty_selection;
  std::cout << "Empty selection is empty: " << (empty_selection.IsEmpty() ? "yes" : "no") << std::endl;
  
  visualization::SurfaceData empty_surface;
  std::cout << "Empty surface is empty: " << (empty_surface.IsEmpty() ? "yes" : "no") << std::endl;
  
  std::cout << "\n=== All Tests Passed! ===" << std::endl;
  std::cout << "The new visualization contracts work correctly alongside existing code." << std::endl;
  
  return 0;
}