/**
 * @file selection_test_utils.hpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-29
 * @brief Shared utilities for selection testing across multiple renderable types
 *
 * This header provides common functionality for testing selection features
 * across different renderable object types (spheres, point clouds, line strips,
 * meshes, etc.). It includes shared UI components, test setup helpers, and
 * common interaction patterns.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#ifndef QUICKVIZ_SELECTION_TEST_UTILS_HPP
#define QUICKVIZ_SELECTION_TEST_UTILS_HPP

#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <functional>

#include "imview/viewer.hpp"
#include "imview/box.hpp"
#include "imview/panel.hpp"
#include "imview/styling.hpp"

#include "gldraw/gl_scene_panel.hpp"
#include "gldraw/selection_manager.hpp"
#include "gldraw/renderable/grid.hpp"

namespace quickviz {
namespace selection_test_utils {

/**
 * @brief Info panel showing current selection details
 * 
 * Displays information about the currently selected objects/points,
 * multi-selection statistics, and provides selection mode controls.
 */
class SelectionInfoPanel : public Panel {
 public:
  SelectionInfoPanel(const std::string& title, GlScenePanel& scene_panel);

  void SetLastSelection(const SelectionResult& result);
  void UpdateMultiSelection(const MultiSelection& multi);
  void Draw() override;

 private:
  void DrawSelectionInfo();
  void DrawMultiSelectionInfo();
  void DrawSelectionControls();

  GlScenePanel& scene_panel_;
  SelectionResult last_selection_;
  size_t multi_selection_count_ = 0;
  size_t point_count_ = 0;
  size_t object_count_ = 0;
  glm::vec3 selection_centroid_{0.0f};
};

/**
 * @brief Enhanced scene panel with selection interaction handling
 * 
 * Extends GlScenePanel with mouse and keyboard handling for selection testing.
 * Supports various selection modes, multi-selection patterns, and keyboard shortcuts.
 */
class SelectionDemoPanel : public GlScenePanel {
 public:
  SelectionDemoPanel(const std::string& title);

  void SetSelectionCallback(std::function<void(const SelectionResult&, const MultiSelection&)> callback);
  void Draw() override;

 private:
  void HandleMouseSelection();
  void HandleKeyboardShortcuts();

  std::function<void(const SelectionResult&, const MultiSelection&)> selection_callback_;
  SelectionMode selection_mode_ = SelectionMode::kHybrid;
};

/**
 * @brief Test application base class for selection testing
 * 
 * Provides common setup and teardown for selection tests, including
 * viewer creation, panel setup, and basic scene configuration.
 */
class SelectionTestApp {
 public:
  SelectionTestApp(const std::string& title);
  virtual ~SelectionTestApp() = default;

  // Setup methods to be implemented by derived classes
  virtual void SetupTestObjects(GlSceneManager* scene_manager) = 0;
  virtual std::string GetTestDescription() const = 0;
  virtual std::string GetInstructions() const = 0;

  // Common functionality
  void AddReferenceGrid(GlSceneManager* scene_manager, float size = 10.0f, float spacing = 1.0f);
  void PrintTestHeader();
  void PrintInstructions();
  int Run();

 protected:
  std::string title_;
  std::unique_ptr<Viewer> viewer_;
  std::shared_ptr<SelectionDemoPanel> scene_panel_;
  std::shared_ptr<SelectionInfoPanel> info_panel_;
  std::shared_ptr<Box> main_container_;
};

/**
 * @brief Utility functions for creating test objects
 */
namespace TestObjectFactory {
  
  /**
   * @brief Create a grid of test spheres with systematic colors
   * @param scene_manager Scene to add spheres to
   * @param positions Grid positions to place spheres
   * @param prefix Name prefix for sphere objects
   * @param radius Sphere radius
   */
  void CreateSphereGrid(GlSceneManager* scene_manager,
                       const std::vector<glm::vec3>& positions,
                       const std::string& prefix = "sphere",
                       float radius = 1.0f);

  /**
   * @brief Create test point clouds with different patterns
   * @param scene_manager Scene to add point clouds to
   */
  void CreateTestPointClouds(GlSceneManager* scene_manager);
  
  /**
   * @brief Create test line strips with various patterns
   * @param scene_manager Scene to add line strips to
   */
  void CreateTestLineStrips(GlSceneManager* scene_manager);
  
  /**
   * @brief Create test meshes for area selection
   * @param scene_manager Scene to add meshes to
   */
  void CreateTestMeshes(GlSceneManager* scene_manager);

  /**
   * @brief Create test cylinders for connection visualization
   * @param scene_manager Scene to add cylinders to
   */
  void CreateTestCylinders(GlSceneManager* scene_manager);
}

/**
 * @brief Helper functions for test setup and validation
 */
namespace TestHelpers {
  
  /**
   * @brief Generate systematic colors for visual identification
   * @param count Number of colors to generate
   * @param base_hue Base hue for color variation
   */
  std::vector<glm::vec3> GenerateTestColors(size_t count, float base_hue = 0.0f);
  
  /**
   * @brief Create evenly spaced 3D grid positions
   * @param dimensions Grid dimensions (x, y, z)
   * @param spacing Distance between grid points
   * @param center Grid center position
   */
  std::vector<glm::vec3> GenerateGridPositions(const glm::ivec3& dimensions,
                                              float spacing = 2.0f,
                                              const glm::vec3& center = glm::vec3(0.0f));
  
  /**
   * @brief Print detailed object information for debugging
   * @param objects List of object names and positions
   */
  void PrintObjectDetails(const std::vector<std::pair<std::string, glm::vec3>>& objects);
}

} // namespace selection_test_utils
} // namespace quickviz

#endif // QUICKVIZ_SELECTION_TEST_UTILS_HPP