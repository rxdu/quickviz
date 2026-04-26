/**
 * @file test_line_strip_selection.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-29
 * @brief Test for LineStrip object selection functionality
 *
 * This test validates the newly implemented LineStrip selection support:
 * - LineStrip highlighting (yellow color, increased width)
 * - Bounding box calculation for different line patterns
 * - Multi-selection with various line types
 * - Performance with complex polylines and paths
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "selection_test_utils.hpp"
#include "gldraw/renderable/line_strip.hpp"
#include <random>
#include <cmath>

using namespace quickviz;
using namespace selection_test_utils;

/**
 * @brief Test application for LineStrip selection functionality
 */
class LineStripSelectionTest : public SelectionTestApp {
 public:
  LineStripSelectionTest() : SelectionTestApp("LineStrip Selection Test") {}

  void SetupTestObjects(SceneManager* scene_manager) override {
    SetupBasicLineStrips(scene_manager);
    SetupMathematicalCurves(scene_manager);
    SetupRobotPaths(scene_manager);
    SetupComplexPolylines(scene_manager);
  }

  std::string GetTestDescription() const override {
    return "Interactive test for LineStrip selection functionality.\n"
           "Tests the newly implemented selection support with highlighting,\n"
           "bounding box calculation, and multi-selection for various line patterns.";
  }

  std::string GetInstructions() const override {
    return "=== Mouse Controls ===\n"
           "- Left Click: Select LineStrip (turns yellow, 2x width)\n"
           "- Ctrl+Shift+Click: Add LineStrip to selection\n"
           "- Ctrl+Alt+Click: Toggle LineStrip selection\n"
           "- Ctrl+Right Click: Clear all selections\n"
           "\n"
           "=== Keyboard Shortcuts ===\n"
           "- O: Object selection mode (lines only)\n"
           "- H: Hybrid selection mode (default)\n"
           "- C: Clear selection\n"
           "\n"
           "=== Test Features ===\n"
           "- NEW: LineStrip selection support\n"
           "- Visual highlighting (yellow + thicker lines)\n"
           "- Accurate bounding box calculation\n"
           "- Various line patterns and complexities\n"
           "- Performance with multi-segment polylines";
  }

 private:
  void SetupBasicLineStrips(SceneManager* scene_manager) {
    // Simple geometric patterns for basic selection testing
    
    // 1. Straight horizontal line
    std::vector<glm::vec3> horizontal_points = {
        glm::vec3(-6.0f, -8.0f, 1.0f),
        glm::vec3(-2.0f, -8.0f, 1.0f),
        glm::vec3(2.0f, -8.0f, 1.0f),
        glm::vec3(6.0f, -8.0f, 1.0f)
    };
    auto horizontal_line = std::make_unique<LineStrip>();
    horizontal_line->SetPoints(horizontal_points);
    horizontal_line->SetColor(glm::vec3(1.0f, 0.2f, 0.2f));  // Red
    horizontal_line->SetLineWidth(3.0f);
    scene_manager->AddOpenGLObject("horizontal_line", std::move(horizontal_line));

    // 2. L-shaped path
    std::vector<glm::vec3> l_shape_points = {
        glm::vec3(-8.0f, -4.0f, 1.0f),
        glm::vec3(-8.0f, 0.0f, 1.0f),
        glm::vec3(-4.0f, 0.0f, 1.0f)
    };
    auto l_shape = std::make_unique<LineStrip>();
    l_shape->SetPoints(l_shape_points);
    l_shape->SetColor(glm::vec3(0.2f, 1.0f, 0.2f));  // Green
    l_shape->SetLineWidth(4.0f);
    scene_manager->AddOpenGLObject("l_shape_path", std::move(l_shape));

    // 3. Triangle (closed)
    std::vector<glm::vec3> triangle_points = {
        glm::vec3(4.0f, -4.0f, 1.0f),
        glm::vec3(8.0f, -4.0f, 1.0f),
        glm::vec3(6.0f, 0.0f, 1.0f)
    };
    auto triangle = std::make_unique<LineStrip>();
    triangle->SetPoints(triangle_points);
    triangle->SetColor(glm::vec3(0.2f, 0.2f, 1.0f));  // Blue
    triangle->SetLineWidth(3.5f);
    triangle->SetClosed(true);  // Closed triangle
    scene_manager->AddOpenGLObject("triangle_path", std::move(triangle));

    std::cout << "✓ Created basic line strips: horizontal, L-shape, triangle" << std::endl;
  }

  void SetupMathematicalCurves(SceneManager* scene_manager) {
    // 1. Sine wave
    std::vector<glm::vec3> sine_points;
    const int sine_segments = 60;
    for (int i = 0; i <= sine_segments; ++i) {
      float t = static_cast<float>(i) / sine_segments;
      float x = -8.0f + t * 16.0f;  // -8 to 8
      float y = 4.0f + 2.5f * sin(t * 6.283f * 2.0f);  // 2 full waves
      sine_points.emplace_back(x, y, 2.0f);
    }
    auto sine_curve = std::make_unique<LineStrip>();
    sine_curve->SetPoints(sine_points);
    sine_curve->SetColor(glm::vec3(1.0f, 0.5f, 0.0f));  // Orange
    sine_curve->SetLineWidth(2.5f);
    scene_manager->AddOpenGLObject("sine_curve", std::move(sine_curve));

    // 2. Spiral
    std::vector<glm::vec3> spiral_points;
    const int spiral_segments = 80;
    const float max_radius = 4.0f;
    for (int i = 0; i <= spiral_segments; ++i) {
      float t = static_cast<float>(i) / spiral_segments;
      float angle = t * 6.283f * 3.0f;  // 3 full rotations
      float radius = max_radius * t;
      spiral_points.emplace_back(
          radius * cos(angle),
          8.0f + radius * sin(angle),
          3.0f
      );
    }
    auto spiral_curve = std::make_unique<LineStrip>();
    spiral_curve->SetPoints(spiral_points);
    spiral_curve->SetColor(glm::vec3(0.8f, 0.2f, 0.8f));  // Magenta
    spiral_curve->SetLineWidth(2.0f);
    scene_manager->AddOpenGLObject("spiral_curve", std::move(spiral_curve));

    std::cout << "✓ Created mathematical curves: sine wave, spiral" << std::endl;
  }

  void SetupRobotPaths(SceneManager* scene_manager) {
    // Simulate realistic robot/vehicle paths
    
    // 1. Navigation waypoints
    std::vector<glm::vec3> nav_waypoints = {
        glm::vec3(-12.0f, -10.0f, 0.5f),  // Start
        glm::vec3(-8.0f, -10.0f, 0.5f),   // Move right
        glm::vec3(-8.0f, -6.0f, 0.5f),    // Turn north
        glm::vec3(-4.0f, -6.0f, 0.5f),    // Continue east
        glm::vec3(-4.0f, -2.0f, 0.5f),    // Turn north again
        glm::vec3(0.0f, -2.0f, 0.5f),     // Final approach
        glm::vec3(0.0f, 2.0f, 0.5f)       // Goal
    };
    auto nav_path = std::make_unique<LineStrip>();
    nav_path->SetPoints(nav_waypoints);
    nav_path->SetColor(glm::vec3(0.0f, 0.8f, 0.8f));  // Cyan
    nav_path->SetLineWidth(4.0f);
    nav_path->SetShowPoints(true, 8.0f);  // Show waypoints
    scene_manager->AddOpenGLObject("navigation_path", std::move(nav_path));

    // 2. Curved trajectory (smooth path)
    std::vector<glm::vec3> smooth_traj;
    const int traj_points = 50;
    for (int i = 0; i <= traj_points; ++i) {
      float t = static_cast<float>(i) / traj_points;
      // Bezier-like curve
      float x = 6.0f + t * 8.0f + 2.0f * sin(t * 3.14159f);
      float y = -10.0f + t * 12.0f - 1.5f * cos(t * 3.14159f * 2.0f);
      smooth_traj.emplace_back(x, y, 0.5f);
    }
    auto smooth_path = std::make_unique<LineStrip>();
    smooth_path->SetPoints(smooth_traj);
    smooth_path->SetColor(glm::vec3(1.0f, 1.0f, 0.2f));  // Yellow
    smooth_path->SetLineWidth(3.0f);
    scene_manager->AddOpenGLObject("smooth_trajectory", std::move(smooth_path));

    std::cout << "✓ Created robot paths: navigation waypoints, smooth trajectory" << std::endl;
  }

  void SetupComplexPolylines(SceneManager* scene_manager) {
    // Complex multi-segment polylines for performance testing
    
    // 1. Random walk path
    std::vector<glm::vec3> random_walk;
    std::mt19937 rng(456);
    std::uniform_real_distribution<float> step_dist(-1.5f, 1.5f);
    
    glm::vec3 current_pos(10.0f, -8.0f, 4.0f);
    random_walk.push_back(current_pos);
    
    const int walk_steps = 40;
    for (int i = 0; i < walk_steps; ++i) {
      current_pos += glm::vec3(
          step_dist(rng),
          step_dist(rng),
          step_dist(rng) * 0.3f  // Less variation in Z
      );
      random_walk.push_back(current_pos);
    }
    
    auto walk_path = std::make_unique<LineStrip>();
    walk_path->SetPoints(random_walk);
    walk_path->SetColor(glm::vec3(0.6f, 0.9f, 0.3f));  // Lime green
    walk_path->SetLineWidth(2.0f);
    scene_manager->AddOpenGLObject("random_walk", std::move(walk_path));

    // 2. Complex boundary contour
    std::vector<glm::vec3> boundary_contour;
    const int contour_points = 60;
    for (int i = 0; i < contour_points; ++i) {
      float angle = (static_cast<float>(i) / contour_points) * 6.283f;
      
      // Complex shape with multiple harmonics
      float r1 = 3.0f + 1.5f * cos(angle * 3.0f);
      float r2 = r1 + 0.8f * sin(angle * 7.0f);
      
      boundary_contour.emplace_back(
          -15.0f + r2 * cos(angle),
          4.0f + r2 * sin(angle),
          2.5f
      );
    }
    
    auto boundary = std::make_unique<LineStrip>();
    boundary->SetPoints(boundary_contour);
    boundary->SetColor(glm::vec3(0.9f, 0.3f, 0.6f));  // Pink
    boundary->SetLineWidth(2.5f);
    boundary->SetClosed(true);  // Closed boundary
    scene_manager->AddOpenGLObject("complex_boundary", std::move(boundary));

    std::cout << "✓ Created complex polylines: random walk (" << random_walk.size() 
              << " points), complex boundary (" << boundary_contour.size() << " points)" << std::endl;
  }
};

int main() {
  LineStripSelectionTest app;
  return app.Run();
}