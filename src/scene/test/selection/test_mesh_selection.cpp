/**
 * @file test_mesh_selection.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-29
 * @brief Test for Mesh object selection functionality
 *
 * This test validates the newly implemented Mesh selection support:
 * - Mesh highlighting (yellow wireframe outline)
 * - Bounding box calculation for triangle meshes
 * - Multi-selection with different mesh shapes
 * - Performance with complex geometry
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "selection_test_utils.hpp"
#include "scene/renderable/mesh.hpp"
#include <random>
#include <cmath>

using namespace quickviz;
using namespace selection_test_utils;

/**
 * @brief Test application for Mesh selection functionality
 */
class MeshSelectionTest : public SelectionTestApp {
 public:
  MeshSelectionTest() : SelectionTestApp("Mesh Selection Test") {}

  void SetupTestObjects(SceneManager* scene_manager) override {
    SetupBasicMeshes(scene_manager);
    SetupGeometricShapes(scene_manager);
    SetupComplexMeshes(scene_manager);
    SetupTerrainPatches(scene_manager);
  }

  std::string GetTestDescription() const override {
    return "Interactive test for Mesh selection functionality.\n"
           "Tests the newly implemented selection support with wireframe highlighting,\n"
           "bounding box calculation, and multi-selection for various mesh types.";
  }

  std::string GetInstructions() const override {
    return "=== Mouse Controls ===\n"
           "- Left Click: Select Mesh (yellow wireframe outline)\n"
           "- Ctrl+Shift+Click: Add Mesh to selection\n"
           "- Ctrl+Alt+Click: Toggle Mesh selection\n"
           "- Ctrl+Right Click: Clear all selections\n"
           "\n"
           "=== Keyboard Shortcuts ===\n"
           "- O: Object selection mode (meshes only)\n"
           "- H: Hybrid selection mode (default)\n"
           "- C: Clear selection\n"
           "\n"
           "=== Test Features ===\n"
           "- NEW: Mesh selection support\n"
           "- Visual highlighting (yellow wireframe outline)\n"
           "- Accurate bounding box from vertices\n"
           "- Various mesh types and complexities\n"
           "- Performance with high vertex counts";
  }

 private:
  void SetupBasicMeshes(SceneManager* scene_manager) {
    // Simple quad mesh
    std::vector<glm::vec3> quad_vertices = {
        glm::vec3(-2.0f, -1.0f, 1.0f),  // Bottom left
        glm::vec3(2.0f, -1.0f, 1.0f),   // Bottom right
        glm::vec3(2.0f, 1.0f, 1.0f),    // Top right
        glm::vec3(-2.0f, 1.0f, 1.0f)    // Top left
    };
    std::vector<uint32_t> quad_indices = {
        0, 1, 2,  // First triangle
        2, 3, 0   // Second triangle
    };
    
    auto quad_mesh = std::make_unique<Mesh>();
    quad_mesh->SetVertices(quad_vertices);
    quad_mesh->SetIndices(quad_indices);
    quad_mesh->SetColor(glm::vec3(0.8f, 0.3f, 0.3f));  // Red
    scene_manager->AddOpenGLObject("quad_mesh", std::move(quad_mesh));

    // Simple triangle mesh
    std::vector<glm::vec3> tri_vertices = {
        glm::vec3(-1.0f, -8.0f, 1.0f),
        glm::vec3(1.0f, -8.0f, 1.0f),
        glm::vec3(0.0f, -6.0f, 1.0f)
    };
    std::vector<uint32_t> tri_indices = {0, 1, 2};
    
    auto triangle_mesh = std::make_unique<Mesh>();
    triangle_mesh->SetVertices(tri_vertices);
    triangle_mesh->SetIndices(tri_indices);
    triangle_mesh->SetColor(glm::vec3(0.3f, 0.8f, 0.3f));  // Green
    scene_manager->AddOpenGLObject("triangle_mesh", std::move(triangle_mesh));

    std::cout << "✓ Created basic meshes: quad, triangle" << std::endl;
  }

  void SetupGeometricShapes(SceneManager* scene_manager) {
    // Hexagon mesh
    std::vector<glm::vec3> hex_vertices;
    std::vector<uint32_t> hex_indices;
    
    const int hex_sides = 6;
    const float hex_radius = 2.0f;
    const glm::vec3 hex_center(6.0f, 0.0f, 1.0f);
    
    // Center vertex
    hex_vertices.push_back(hex_center);
    
    // Perimeter vertices
    for (int i = 0; i < hex_sides; ++i) {
      float angle = (i * 2.0f * M_PI) / hex_sides;
      hex_vertices.emplace_back(
          hex_center.x + hex_radius * cos(angle),
          hex_center.y + hex_radius * sin(angle),
          hex_center.z
      );
    }
    
    // Triangle indices (fan from center)
    for (int i = 0; i < hex_sides; ++i) {
      hex_indices.push_back(0);  // Center
      hex_indices.push_back(i + 1);
      hex_indices.push_back(((i + 1) % hex_sides) + 1);
    }
    
    auto hexagon_mesh = std::make_unique<Mesh>();
    hexagon_mesh->SetVertices(hex_vertices);
    hexagon_mesh->SetIndices(hex_indices);
    hexagon_mesh->SetColor(glm::vec3(0.3f, 0.3f, 0.8f));  // Blue
    scene_manager->AddOpenGLObject("hexagon_mesh", std::move(hexagon_mesh));

    // Star shape mesh
    std::vector<glm::vec3> star_vertices;
    std::vector<uint32_t> star_indices;
    
    const int star_points = 5;
    const float outer_radius = 2.5f;
    const float inner_radius = 1.0f;
    const glm::vec3 star_center(-6.0f, 6.0f, 1.0f);
    
    // Center vertex
    star_vertices.push_back(star_center);
    
    // Alternating outer and inner vertices
    for (int i = 0; i < star_points * 2; ++i) {
      float angle = (i * M_PI) / star_points;
      float radius = (i % 2 == 0) ? outer_radius : inner_radius;
      star_vertices.emplace_back(
          star_center.x + radius * cos(angle),
          star_center.y + radius * sin(angle),
          star_center.z
      );
    }
    
    // Triangle indices
    for (int i = 0; i < star_points * 2; ++i) {
      star_indices.push_back(0);  // Center
      star_indices.push_back(i + 1);
      star_indices.push_back(((i + 1) % (star_points * 2)) + 1);
    }
    
    auto star_mesh = std::make_unique<Mesh>();
    star_mesh->SetVertices(star_vertices);
    star_mesh->SetIndices(star_indices);
    star_mesh->SetColor(glm::vec3(0.8f, 0.8f, 0.3f));  // Yellow
    scene_manager->AddOpenGLObject("star_mesh", std::move(star_mesh));

    std::cout << "✓ Created geometric shapes: hexagon, star" << std::endl;
  }

  void SetupComplexMeshes(SceneManager* scene_manager) {
    // Subdivided plane (grid mesh)
    std::vector<glm::vec3> grid_vertices;
    std::vector<uint32_t> grid_indices;
    
    const int grid_res = 8;  // 8x8 grid
    const float grid_size = 4.0f;
    const glm::vec3 grid_offset(0.0f, 8.0f, 1.0f);
    
    // Generate vertices
    for (int y = 0; y <= grid_res; ++y) {
      for (int x = 0; x <= grid_res; ++x) {
        float fx = (x / float(grid_res) - 0.5f) * grid_size;
        float fy = (y / float(grid_res) - 0.5f) * grid_size;
        
        // Add some height variation for visual interest
        float height = 0.3f * sin(fx * 2.0f) * cos(fy * 2.0f);
        
        grid_vertices.emplace_back(
            grid_offset.x + fx,
            grid_offset.y + fy,
            grid_offset.z + height
        );
      }
    }
    
    // Generate triangle indices
    for (int y = 0; y < grid_res; ++y) {
      for (int x = 0; x < grid_res; ++x) {
        int base = y * (grid_res + 1) + x;
        
        // Two triangles per grid cell
        grid_indices.push_back(base);
        grid_indices.push_back(base + 1);
        grid_indices.push_back(base + grid_res + 1);
        
        grid_indices.push_back(base + 1);
        grid_indices.push_back(base + grid_res + 2);
        grid_indices.push_back(base + grid_res + 1);
      }
    }
    
    auto grid_mesh = std::make_unique<Mesh>();
    grid_mesh->SetVertices(grid_vertices);
    grid_mesh->SetIndices(grid_indices);
    grid_mesh->SetColor(glm::vec3(0.6f, 0.4f, 0.8f));  // Purple
    scene_manager->AddOpenGLObject("grid_mesh", std::move(grid_mesh));

    std::cout << "✓ Created complex mesh: subdivided grid (" << grid_vertices.size() 
              << " vertices, " << grid_indices.size()/3 << " triangles)" << std::endl;
  }

  void SetupTerrainPatches(SceneManager* scene_manager) {
    // Irregular terrain patch
    std::vector<glm::vec3> terrain_vertices;
    std::vector<uint32_t> terrain_indices;
    
    std::mt19937 rng(789);
    std::uniform_real_distribution<float> height_dist(0.0f, 1.5f);
    std::uniform_real_distribution<float> pos_noise(-0.2f, 0.2f);
    
    const int patch_res = 6;
    const float patch_size = 3.0f;
    const glm::vec3 patch_offset(10.0f, 6.0f, 1.0f);
    
    // Generate irregular vertex positions
    for (int y = 0; y <= patch_res; ++y) {
      for (int x = 0; x <= patch_res; ++x) {
        float fx = (x / float(patch_res) - 0.5f) * patch_size;
        float fy = (y / float(patch_res) - 0.5f) * patch_size;
        
        // Add position noise for irregular shape
        if (x > 0 && x < patch_res && y > 0 && y < patch_res) {
          fx += pos_noise(rng);
          fy += pos_noise(rng);
        }
        
        terrain_vertices.emplace_back(
            patch_offset.x + fx,
            patch_offset.y + fy,
            patch_offset.z + height_dist(rng)
        );
      }
    }
    
    // Generate indices (same as grid)
    for (int y = 0; y < patch_res; ++y) {
      for (int x = 0; x < patch_res; ++x) {
        int base = y * (patch_res + 1) + x;
        
        terrain_indices.push_back(base);
        terrain_indices.push_back(base + 1);
        terrain_indices.push_back(base + patch_res + 1);
        
        terrain_indices.push_back(base + 1);
        terrain_indices.push_back(base + patch_res + 2);
        terrain_indices.push_back(base + patch_res + 1);
      }
    }
    
    auto terrain_mesh = std::make_unique<Mesh>();
    terrain_mesh->SetVertices(terrain_vertices);
    terrain_mesh->SetIndices(terrain_indices);
    terrain_mesh->SetColor(glm::vec3(0.5f, 0.7f, 0.3f));  // Green terrain
    scene_manager->AddOpenGLObject("terrain_patch", std::move(terrain_mesh));

    // Circular area mesh
    std::vector<glm::vec3> circle_vertices;
    std::vector<uint32_t> circle_indices;
    
    const int circle_segments = 16;
    const float circle_radius = 2.0f;
    const glm::vec3 circle_center(-10.0f, -6.0f, 1.0f);
    
    // Center vertex
    circle_vertices.push_back(circle_center);
    
    // Perimeter vertices
    for (int i = 0; i < circle_segments; ++i) {
      float angle = (i * 2.0f * M_PI) / circle_segments;
      circle_vertices.emplace_back(
          circle_center.x + circle_radius * cos(angle),
          circle_center.y + circle_radius * sin(angle),
          circle_center.z
      );
    }
    
    // Triangle fan indices
    for (int i = 0; i < circle_segments; ++i) {
      circle_indices.push_back(0);  // Center
      circle_indices.push_back(i + 1);
      circle_indices.push_back(((i + 1) % circle_segments) + 1);
    }
    
    auto circle_mesh = std::make_unique<Mesh>();
    circle_mesh->SetVertices(circle_vertices);
    circle_mesh->SetIndices(circle_indices);
    circle_mesh->SetColor(glm::vec3(0.8f, 0.5f, 0.2f));  // Orange
    scene_manager->AddOpenGLObject("circular_area", std::move(circle_mesh));

    std::cout << "✓ Created terrain patches: irregular terrain (" << terrain_vertices.size() 
              << " vertices), circular area (" << circle_vertices.size() << " vertices)" << std::endl;
  }
};

int main() {
  MeshSelectionTest app;
  return app.Run();
}