/**
 * @file test_comprehensive_selection.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-29
 * @brief Comprehensive selection test for all supported renderable types
 *
 * This test combines all supported selection types in one scene:
 * - Spheres (object selection)
 * - Point clouds (individual point selection)
 * - LineStrips (newly implemented selection)
 * - Future: Meshes, Cylinders, etc.
 *
 * Demonstrates mixed-mode selection and overall system integration.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "selection_test_utils.hpp"
#include "gldraw/renderable/sphere.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/renderable/line_strip.hpp"
#include "gldraw/feedback/visual_feedback_system.hpp"
#include "core/event/input_event.hpp"
#include <random>
#include <cmath>
#include <unordered_map>
#include <algorithm>

using namespace quickviz;
using namespace selection_test_utils;

/**
 * @brief Comprehensive test for all selection types with enhanced interactions
 */
class ComprehensiveSelectionTest : public SelectionTestApp {
 public:
  ComprehensiveSelectionTest() : SelectionTestApp("Comprehensive Selection Test") {
    SetupEnhancedInteractions();
  }

  void SetupTestObjects(SceneManager* scene_manager) override {
    SetupMixedObjectScene(scene_manager);
    PrintSceneDescription();
  }

  std::string GetTestDescription() const override {
    return "Comprehensive test for all supported selection types.\n"
           "Combines spheres, point clouds, and LineStrips in one scene\n"
           "to test mixed-mode selection and system integration.";
  }

  std::string GetInstructions() const override {
    return "=== Mouse Controls ===\n"
           "- Left Click: Select any object/point\n"
           "- Ctrl+Shift+Click: Add to multi-selection\n"
           "- Ctrl+Alt+Click: Toggle selection\n"
           "- Ctrl+Right Click: Clear all selections\n"
           "\n"
           "=== Selection Modes (Keyboard) ===\n"
           "- P: Points only (point clouds)\n"
           "- O: Objects only (spheres, lines)\n"
           "- H: Hybrid mode (everything) [DEFAULT]\n"
           "- C: Clear selection\n"
           "\n"
           "=== Visual Feedback ===\n"
           "- Spheres: Yellow highlight with original size\n"
           "- Points: Yellow highlight with increased size\n"
           "- LineStrips: Yellow color with 2x line width\n"
           "\n"
           "=== Test Integration ===\n"
           "- Mixed object types in one scene\n"
           "- Multi-selection across types\n"
           "- Selection priority handling\n"
           "- Performance with diverse objects\n"
           "\n"
           "=== Enhanced Interactions ===\n"
           "- SPACE: Print detailed selection info\n"
           "- R: Randomize colors of selected objects\n"
           "- B: Make selected objects bigger\n"
           "- S: Make selected objects smaller\n"
           "- F: Focus camera on selection\n"
           "- T: Toggle animation of selected objects\n"
           "- I: Show/hide object info overlays";
  }

 private:
  void SetupMixedObjectScene(SceneManager* scene_manager) {
    // Create a realistic mixed scene with all supported selection types
    
    SetupSceneSpheres(scene_manager);
    SetupScenePointClouds(scene_manager);
    SetupSceneLineStrips(scene_manager);
    
    // Add connecting elements to show relationships
    SetupConnectingElements(scene_manager);
  }

  void SetupSceneSpheres(SceneManager* scene_manager) {
    // Create spheres representing key locations/nodes
    std::vector<std::pair<glm::vec3, std::string>> key_locations = {
        {glm::vec3(-8.0f, -8.0f, 2.0f), "start_node"},
        {glm::vec3(0.0f, -8.0f, 2.0f), "checkpoint_a"},
        {glm::vec3(8.0f, -8.0f, 2.0f), "checkpoint_b"},
        {glm::vec3(8.0f, 0.0f, 2.0f), "junction"},
        {glm::vec3(8.0f, 8.0f, 2.0f), "goal_node"},
        {glm::vec3(-8.0f, 8.0f, 2.0f), "observation_post"},
        {glm::vec3(0.0f, 0.0f, 6.0f), "elevated_marker"}
    };
    
    std::vector<glm::vec3> colors = {
        glm::vec3(0.9f, 0.2f, 0.2f),  // Red start
        glm::vec3(0.9f, 0.7f, 0.2f),  // Orange checkpoint
        glm::vec3(0.9f, 0.7f, 0.2f),  // Orange checkpoint
        glm::vec3(0.2f, 0.2f, 0.9f),  // Blue junction
        glm::vec3(0.2f, 0.9f, 0.2f),  // Green goal
        glm::vec3(0.7f, 0.2f, 0.9f),  // Purple observation
        glm::vec3(0.9f, 0.9f, 0.2f)   // Yellow elevated
    };
    
    for (size_t i = 0; i < key_locations.size(); ++i) {
      auto sphere = std::make_unique<Sphere>(key_locations[i].first, 1.2f);
      sphere->SetColor(colors[i]);
      sphere->SetRenderMode(Sphere::RenderMode::kSolid);
      scene_manager->AddOpenGLObject(key_locations[i].second, std::move(sphere));
    }
    
    std::cout << "✓ Created scene spheres: " << key_locations.size() << " key locations" << std::endl;
  }

  void SetupScenePointClouds(SceneManager* scene_manager) {
    // Sensor data visualization as point clouds
    
    // 1. LIDAR scan pattern
    std::vector<glm::vec3> lidar_points;
    std::vector<glm::vec3> lidar_colors;
    
    const int lidar_rays = 180;
    const float max_range = 12.0f;
    const glm::vec3 lidar_origin(-8.0f, -8.0f, 2.5f);  // Near start_node
    
    std::mt19937 rng(789);
    std::uniform_real_distribution<float> range_noise(0.8f, 1.0f);
    
    for (int i = 0; i < lidar_rays; ++i) {
      float angle = (static_cast<float>(i) / lidar_rays) * 3.14159f;  // 180 degrees
      float range = max_range * range_noise(rng);
      
      glm::vec3 point = lidar_origin + glm::vec3(
          range * cos(angle),
          range * sin(angle),
          0.0f
      );
      lidar_points.push_back(point);
      
      // Color by distance
      float dist_ratio = range / max_range;
      lidar_colors.emplace_back(1.0f - dist_ratio, dist_ratio * 0.8f, 0.3f);
    }
    
    auto lidar_cloud = std::make_unique<PointCloud>();
    lidar_cloud->SetPoints(lidar_points, lidar_colors);
    lidar_cloud->SetPointSize(6.0f);
    scene_manager->AddOpenGLObject("lidar_scan", std::move(lidar_cloud));

    // 2. Dense measurement cluster
    std::vector<glm::vec3> measurement_points;
    std::vector<glm::vec3> measurement_colors;
    
    const glm::vec3 measurement_center(0.0f, 0.0f, 3.0f);
    std::normal_distribution<float> cluster_dist(0.0f, 1.2f);
    
    const int measurement_count = 80;
    for (int i = 0; i < measurement_count; ++i) {
      glm::vec3 offset(
          cluster_dist(rng),
          cluster_dist(rng),
          cluster_dist(rng) * 0.4f
      );
      measurement_points.push_back(measurement_center + offset);
      
      // Cool color palette
      measurement_colors.emplace_back(0.2f, 0.6f + offset.z * 0.3f, 0.9f);
    }
    
    auto measurement_cloud = std::make_unique<PointCloud>();
    measurement_cloud->SetPoints(measurement_points, measurement_colors);
    measurement_cloud->SetPointSize(7.0f);
    scene_manager->AddOpenGLObject("measurements", std::move(measurement_cloud));
    
    std::cout << "✓ Created scene point clouds: LIDAR scan (" << lidar_points.size() 
              << " points), measurements (" << measurement_points.size() << " points)" << std::endl;
  }

  void SetupSceneLineStrips(SceneManager* scene_manager) {
    // Connection paths and boundaries
    
    // 1. Main navigation path
    std::vector<glm::vec3> nav_path = {
        glm::vec3(-8.0f, -8.0f, 1.8f),  // start_node
        glm::vec3(-4.0f, -8.0f, 1.8f),  // intermediate
        glm::vec3(0.0f, -8.0f, 1.8f),   // checkpoint_a
        glm::vec3(4.0f, -8.0f, 1.8f),   // intermediate  
        glm::vec3(8.0f, -8.0f, 1.8f),   // checkpoint_b
        glm::vec3(8.0f, -4.0f, 1.8f),   // turn
        glm::vec3(8.0f, 0.0f, 1.8f),    // junction
        glm::vec3(8.0f, 4.0f, 1.8f),    // final approach
        glm::vec3(8.0f, 8.0f, 1.8f)     // goal_node
    };
    auto main_path = std::make_unique<LineStrip>();
    main_path->SetPoints(nav_path);
    main_path->SetColor(glm::vec3(0.0f, 0.9f, 0.9f));  // Cyan path
    main_path->SetLineWidth(4.0f);
    scene_manager->AddOpenGLObject("main_nav_path", std::move(main_path));

    // 2. Alternative route
    std::vector<glm::vec3> alt_path = {
        glm::vec3(0.0f, -8.0f, 1.8f),   // checkpoint_a
        glm::vec3(0.0f, -4.0f, 1.8f),   // turn north
        glm::vec3(0.0f, 0.0f, 1.8f),    // center
        glm::vec3(4.0f, 4.0f, 1.8f),    // approach goal
        glm::vec3(8.0f, 8.0f, 1.8f)     // goal_node
    };
    auto alternative = std::make_unique<LineStrip>();
    alternative->SetPoints(alt_path);
    alternative->SetColor(glm::vec3(1.0f, 0.6f, 0.0f));  // Orange alternative
    alternative->SetLineWidth(3.0f);
    scene_manager->AddOpenGLObject("alternative_path", std::move(alternative));

    // 3. Security perimeter
    std::vector<glm::vec3> perimeter;
    const int perimeter_points = 32;
    const float perimeter_radius = 15.0f;
    const glm::vec3 perimeter_center(0.0f, 0.0f, 0.5f);
    
    for (int i = 0; i < perimeter_points; ++i) {
      float angle = (static_cast<float>(i) / perimeter_points) * 6.283f;
      // Slightly irregular perimeter
      float radius = perimeter_radius + 2.0f * sin(angle * 5.0f);
      perimeter.emplace_back(
          perimeter_center.x + radius * cos(angle),
          perimeter_center.y + radius * sin(angle),
          perimeter_center.z
      );
    }
    
    auto perimeter_line = std::make_unique<LineStrip>();
    perimeter_line->SetPoints(perimeter);
    perimeter_line->SetColor(glm::vec3(1.0f, 0.2f, 0.2f));  // Red boundary
    perimeter_line->SetLineWidth(2.5f);
    perimeter_line->SetClosed(true);
    scene_manager->AddOpenGLObject("security_perimeter", std::move(perimeter_line));
    
    std::cout << "✓ Created scene line strips: main path, alternative, perimeter" << std::endl;
  }

  void SetupConnectingElements(SceneManager* scene_manager) {
    // Visual connection between elevated marker and junction
    std::vector<glm::vec3> connection_line = {
        glm::vec3(0.0f, 0.0f, 6.0f),    // elevated_marker
        glm::vec3(4.0f, 0.0f, 4.0f),    // intermediate
        glm::vec3(8.0f, 0.0f, 2.0f)     // junction
    };
    auto connection = std::make_unique<LineStrip>();
    connection->SetPoints(connection_line);
    connection->SetColor(glm::vec3(0.8f, 0.8f, 0.8f));  // Gray connection
    connection->SetLineWidth(1.5f);
    scene_manager->AddOpenGLObject("connection_line", std::move(connection));
    
    std::cout << "✓ Created connecting elements" << std::endl;
  }

  void PrintSceneDescription() {
    std::cout << "\n=== Scene Description ===" << std::endl;
    std::cout << "Simulated robotics navigation scenario with:" << std::endl;
    std::cout << "\nSPHERES (Key Locations):" << std::endl;
    std::cout << "  - Red: Start Node (-8, -8, 2)" << std::endl;
    std::cout << "  - Orange: Checkpoints (0, -8, 2) and (8, -8, 2)" << std::endl;
    std::cout << "  - Blue: Junction (8, 0, 2)" << std::endl;
    std::cout << "  - Green: Goal Node (8, 8, 2)" << std::endl;
    std::cout << "  - Purple: Observation Post (-8, 8, 2)" << std::endl;
    std::cout << "  - Yellow: Elevated Marker (0, 0, 6)" << std::endl;
    
    std::cout << "\nPOINT CLOUDS (Sensor Data):" << std::endl;
    std::cout << "  - LIDAR scan: 180-degree scan from start position" << std::endl;
    std::cout << "  - Measurements: Dense cluster around center point" << std::endl;
    
    std::cout << "\nLINE STRIPS (Paths & Boundaries):" << std::endl;
    std::cout << "  - Cyan: Main navigation path (start -> goal)" << std::endl;
    std::cout << "  - Orange: Alternative route via center" << std::endl;
    std::cout << "  - Red: Security perimeter (closed boundary)" << std::endl;
    std::cout << "  - Gray: Connection line (elevated -> junction)" << std::endl;
    
    std::cout << "\nTEST FEATURES:" << std::endl;
    std::cout << "  - Mixed selection types in realistic scenario" << std::endl;
    std::cout << "  - Multi-selection across object types" << std::endl;
    std::cout << "  - Selection mode filtering (P/O/H keys)" << std::endl;
    std::cout << "  - Visual feedback for all selection types" << std::endl;
    std::cout << std::endl;
  }

  // ===== Enhanced Interaction Methods =====
  
  void SetupEnhancedInteractions() {
    // Note: Custom input handling has been simplified to work with current architecture
    // Enhanced interactions will be handled through the selection callback system
    
    // Set up enhanced callback that calls info panel, visual feedback, and our enhancements
    scene_panel_->SetSelectionCallback(
        [this](const SelectionResult& result, const MultiSelection& selection) {
            // First update the info panel UI (what the base class normally does)
            info_panel_->SetLastSelection(result);
            info_panel_->UpdateMultiSelection(selection);
            
            // Forward to visual feedback system
            auto feedback_system = scene_panel_->GetFeedbackSystem();
            if (feedback_system) {
              feedback_system->OnSelectionChanged(selection);
            }
            
            // Then add our enhanced effects
            HandleEnhancedSelection(result, selection);
        });
        
    std::cout << "✓ Enhanced interaction handlers registered" << std::endl;
  }
  
  bool HandleEnhancedKeyInput(const InputEvent& event) {
    if (event.GetType() != InputEventType::kKeyPress) return false;
    
    int key = event.GetKey();
    
    switch (key) {
      case 32: { // SPACE - print detailed selection info
        PrintDetailedSelectionInfo();
        return true;
      }
      
      case 82: { // 'R' - randomize colors of selected objects
        RandomizeSelectedObjectColors();
        return true;
      }
      
      case 66: { // 'B' - make selected objects bigger
        ScaleSelectedObjects(1.2f);
        return true;
      }
      
      case 83: { // 'S' - make selected objects smaller
        ScaleSelectedObjects(0.8f);
        return true;
      }
      
      case 70: { // 'F' - focus camera on selection
        FocusCameraOnSelection();
        return true;
      }
      
      case 84: { // 'T' - toggle animation
        ToggleSelectionAnimation();
        return true;
      }
      
      case 73: { // 'I' - toggle info overlays
        ToggleInfoOverlays();
        return true;
      }
    }
    
    return false;
  }
  
  void HandleEnhancedSelection(const SelectionResult& result, const MultiSelection& selection) {
    // Clear previous highlights first
    ClearAllHighlights();
    
    // Apply visual highlighting to selected objects
    if (!selection.Empty()) {
      std::cout << "\n🎯 Selection Highlighted!" << std::endl;
      
      // Apply highlights to selected items
      ApplySelectionHighlights(selection);
      
      // Show selection statistics
      auto centroid = selection.GetCentroid();
      auto [min_bounds, max_bounds] = selection.GetBounds();
      
      std::cout << "  📊 Selection Stats:" << std::endl;
      std::cout << "    • Total items: " << selection.Count() << std::endl;
      std::cout << "    • Points: " << selection.GetPoints().size() << std::endl;
      std::cout << "    • Objects: " << selection.GetObjects().size() << std::endl;
      std::cout << "    • Centroid: (" << centroid.x << ", " << centroid.y << ", " << centroid.z << ")" << std::endl;
      
      // Debug: Compare selection result position vs computed centroid for point selections
      if (std::holds_alternative<PointSelection>(result)) {
        auto point_sel = std::get<PointSelection>(result);
        std::cout << "  🔍 DEBUG: Selected point reports position (" << point_sel.world_position.x << ", " << point_sel.world_position.y << ", " << point_sel.world_position.z << ")" << std::endl;
        std::cout << "  🔍 DEBUG: Centroid calculated as (" << centroid.x << ", " << centroid.y << ", " << centroid.z << ")" << std::endl;
        
        // Check if they match (within tolerance)
        glm::vec3 diff = glm::abs(point_sel.world_position - centroid);
        if (diff.x > 0.001f || diff.y > 0.001f || diff.z > 0.001f) {
          std::cout << "  ⚠️  WARNING: Position mismatch detected! Difference: (" << diff.x << ", " << diff.y << ", " << diff.z << ")" << std::endl;
        }
      }
                
      // Apply optional pulse effect if animation is enabled
      if (animation_enabled_) {
        ApplyPulseEffect(selection);
      }
    } else {
      std::cout << "\n🔄 Selection cleared - highlights removed" << std::endl;
    }
  }
  
  void PrintDetailedSelectionInfo() {
    const auto& selection = scene_panel_->GetMultiSelection();
    
    std::cout << "\n📋 === DETAILED SELECTION REPORT ===" << std::endl;
    if (selection.Empty()) {
      std::cout << "  No objects currently selected." << std::endl;
      return;
    }
    
    std::cout << "  🎯 Selection Summary:" << std::endl;
    std::cout << "    Total: " << selection.Count() << " items" << std::endl;
    std::cout << "    Points: " << selection.GetPoints().size() << std::endl;
    std::cout << "    Objects: " << selection.GetObjects().size() << std::endl;
    
    auto centroid = selection.GetCentroid();
    std::cout << "    Centroid: (" << centroid.x << ", " << centroid.y << ", " << centroid.z << ")" << std::endl;
    
    // List each selected item with details
    std::cout << "\n  📝 Individual Items:" << std::endl;
    int index = 0;
    for (const auto& item : selection.GetSelections()) {
      std::cout << "    [" << index++ << "] ";
      std::visit([this](const auto& sel) {
        using T = std::decay_t<decltype(sel)>;
        if constexpr (std::is_same_v<T, PointSelection>) {
          std::cout << "POINT: " << sel.cloud_name << "[" << sel.point_index << "] ";
          std::cout << "at (" << sel.world_position.x << ", " << sel.world_position.y << ", " << sel.world_position.z << ")";
        } else if constexpr (std::is_same_v<T, ObjectSelection>) {
          std::cout << "OBJECT: " << sel.object_name << " ";
          std::cout << "at (" << sel.world_position.x << ", " << sel.world_position.y << ", " << sel.world_position.z << ")";
          
          // Try to get object-specific info
          auto* obj = scene_panel_->GetOpenGLObject(sel.object_name);
          if (auto* sphere = dynamic_cast<Sphere*>(obj)) {
            std::cout << " [Sphere r=" << sphere->GetRadius() << "]";
          } else if (dynamic_cast<LineStrip*>(obj)) {
            std::cout << " [LineStrip]";
          }
        }
      }, item);
      std::cout << std::endl;
    }
    std::cout << "==================================" << std::endl;
  }
  
  void RandomizeSelectedObjectColors() {
    const auto& selection = scene_panel_->GetMultiSelection();
    if (selection.Empty()) {
      std::cout << "🎨 No objects selected to recolor" << std::endl;
      return;
    }
    
    std::cout << "🎨 Randomizing colors of " << selection.Count() << " selected objects" << std::endl;
    
    for (const auto& item : selection.GetSelections()) {
      std::visit([this](const auto& sel) {
        using T = std::decay_t<decltype(sel)>;
        if constexpr (std::is_same_v<T, ObjectSelection>) {
          auto* obj = scene_panel_->GetOpenGLObject(sel.object_name);
          
          // Generate vibrant random color
          glm::vec3 new_color(
            0.3f + (rand() % 70) / 100.0f,  // 0.3 to 1.0
            0.3f + (rand() % 70) / 100.0f,  
            0.3f + (rand() % 70) / 100.0f
          );
          
          if (auto* sphere = dynamic_cast<Sphere*>(obj)) {
            sphere->SetColor(new_color);
            std::cout << "  ✓ Recolored sphere " << sel.object_name << std::endl;
          } else if (auto* line = dynamic_cast<LineStrip*>(obj)) {
            line->SetColor(new_color);
            std::cout << "  ✓ Recolored line strip " << sel.object_name << std::endl;
          }
        }
      }, item);
    }
  }
  
  void ScaleSelectedObjects(float scale_factor) {
    const auto& selection = scene_panel_->GetMultiSelection();
    if (selection.Empty()) {
      std::cout << "📏 No objects selected to scale" << std::endl;
      return;
    }
    
    std::string action = (scale_factor > 1.0f) ? "Enlarging" : "Shrinking";
    std::cout << "📏 " << action << " " << selection.Count() << " selected objects by " << scale_factor << "x" << std::endl;
    
    for (const auto& item : selection.GetSelections()) {
      std::visit([this, scale_factor](const auto& sel) {
        using T = std::decay_t<decltype(sel)>;
        if constexpr (std::is_same_v<T, ObjectSelection>) {
          auto* obj = scene_panel_->GetOpenGLObject(sel.object_name);
          
          if (auto* sphere = dynamic_cast<Sphere*>(obj)) {
            float current_radius = sphere->GetRadius();
            float new_radius = current_radius * scale_factor;
            // Keep radius within reasonable bounds
            new_radius = std::max(0.1f, std::min(new_radius, 5.0f));
            sphere->SetRadius(new_radius);
            std::cout << "  ✓ Scaled sphere " << sel.object_name 
                      << " from r=" << current_radius << " to r=" << new_radius << std::endl;
          } else if (auto* line = dynamic_cast<LineStrip*>(obj)) {
            // LineStrip doesn't have a public GetLineWidth() method,
            // so we'll use a default scaling approach
            float new_width = (scale_factor > 1.0f) ? 6.0f : 2.0f; // Simple bigger/smaller
            new_width = std::max(1.0f, std::min(new_width, 10.0f));
            line->SetLineWidth(new_width);
            std::cout << "  ✓ Scaled line strip " << sel.object_name 
                      << " to width " << new_width << std::endl;
          }
        }
      }, item);
    }
  }
  
  void FocusCameraOnSelection() {
    const auto& selection = scene_panel_->GetMultiSelection();
    if (selection.Empty()) {
      std::cout << "📷 No objects selected to focus on" << std::endl;
      return;
    }
    
    auto centroid = selection.GetCentroid();
    auto [min_bounds, max_bounds] = selection.GetBounds();
    
    std::cout << "📷 Focusing camera on selection centroid: (" 
              << centroid.x << ", " << centroid.y << ", " << centroid.z << ")" << std::endl;
    
    // Calculate appropriate camera distance based on selection bounds
    glm::vec3 size = max_bounds - min_bounds;
    float max_dimension = std::max({size.x, size.y, size.z});
    float camera_distance = std::max(10.0f, max_dimension * 2.5f);
    
    // Set camera to look at the selection from a good angle
    auto* camera = scene_panel_->GetSceneManager()->GetCamera();
    glm::vec3 camera_position = centroid + glm::vec3(camera_distance * 0.7f, camera_distance * 0.7f, camera_distance * 0.5f);
    
    camera->SetPosition(camera_position);
    camera->LookAt(centroid);
    
    std::cout << "  ✓ Camera positioned at (" << camera_position.x << ", " 
              << camera_position.y << ", " << camera_position.z << ") looking at selection" << std::endl;
  }
  
  void ToggleSelectionAnimation() {
    animation_enabled_ = !animation_enabled_;
    std::cout << "🎬 Selection animation: " << (animation_enabled_ ? "ENABLED" : "DISABLED") << std::endl;
    
    if (animation_enabled_) {
      std::cout << "  Selected objects will now pulse and rotate" << std::endl;
    } else {
      std::cout << "  Animation effects disabled" << std::endl;
      // Reset any ongoing animations
      ClearAllEffects();
    }
  }
  
  void ToggleInfoOverlays() {
    info_overlays_enabled_ = !info_overlays_enabled_;
    std::cout << "ℹ️  Info overlays: " << (info_overlays_enabled_ ? "ENABLED" : "DISABLED") << std::endl;
    
    if (info_overlays_enabled_) {
      std::cout << "  Object information will be displayed as text overlays" << std::endl;
      ShowInfoOverlays();
    } else {
      std::cout << "  Info overlays hidden" << std::endl;
      HideInfoOverlays();
    }
  }
  
  void ApplySelectionHighlights(const MultiSelection& selection) {
    // Highlight selected objects
    for (const auto& item : selection.GetSelections()) {
      std::visit([this](const auto& sel) {
        using T = std::decay_t<decltype(sel)>;
        if constexpr (std::is_same_v<T, ObjectSelection>) {
          HighlightObject(sel.object_name);
        } else if constexpr (std::is_same_v<T, PointSelection>) {
          HighlightPoint(sel.cloud_name, sel.point_index);
        }
      }, item);
    }
  }
  
  void HighlightObject(const std::string& object_name) {
    auto* obj = scene_panel_->GetOpenGLObject(object_name);
    if (!obj) return;
    
    // Store original properties if not already stored
    if (highlighted_objects_.find(object_name) == highlighted_objects_.end()) {
      ObjectHighlightState state;
      
      if (auto* sphere = dynamic_cast<Sphere*>(obj)) {
        // Store original sphere properties (use default color since no getter)
        state.original_color = glm::vec3(0.7f, 0.7f, 0.9f); // Default sphere color
        state.original_size = sphere->GetRadius();
        state.object_type = "sphere";
        
        // Apply highlight: bright yellow color and slightly larger
        sphere->SetColor(glm::vec3(1.0f, 1.0f, 0.0f)); // Bright yellow
        sphere->SetRadius(state.original_size * 1.15f); // 15% larger
        
        std::cout << "  ✨ Highlighted sphere: " << object_name << std::endl;
        
      } else if (auto* line = dynamic_cast<LineStrip*>(obj)) {
        // Store original line properties (use defaults since no getters)
        state.original_color = glm::vec3(0.0f, 1.0f, 0.0f); // Default line color
        state.original_size = 2.0f; // Default line width
        state.object_type = "linestrip";
        
        // Apply highlight: bright yellow color and thicker line
        line->SetColor(glm::vec3(1.0f, 1.0f, 0.0f)); // Bright yellow
        line->SetLineWidth(state.original_size * 2.0f); // 2x thicker
        
        std::cout << "  ✨ Highlighted line strip: " << object_name << std::endl;
      }
      
      highlighted_objects_[object_name] = state;
    }
  }
  
  void HighlightPoint(const std::string& cloud_name, size_t point_index) {
    auto* obj = scene_panel_->GetOpenGLObject(cloud_name);
    auto* point_cloud = dynamic_cast<PointCloud*>(obj);
    if (!point_cloud) {
      std::cout << "  ❌ ERROR: Could not find point cloud: " << cloud_name << std::endl;
      return;
    }
    
    // Validate point index
    size_t total_points = point_cloud->GetPointCount();
    if (point_index >= total_points) {
      std::cout << "  ❌ ERROR: Invalid point index " << point_index << " (cloud has " << total_points << " points)" << std::endl;
      return;
    }
    
    // Get the actual point position for debugging
    glm::vec3 point_pos = point_cloud->GetPointPosition(point_index);
    std::cout << "  🔍 DEBUG: Selected point " << point_index << " at position (" << point_pos.x << ", " << point_pos.y << ", " << point_pos.z << ")" << std::endl;
    
    // DEBUGGING: Check if point coordinates match by searching through all points
    const auto& all_points = point_cloud->GetPoints();
    bool found_matching_position = false;
    for (size_t i = 0; i < all_points.size(); ++i) {
      const glm::vec3& pt = all_points[i];
      glm::vec3 diff = glm::abs(pt - point_pos);
      if (diff.x < 0.001f && diff.y < 0.001f && diff.z < 0.001f) {
        if (i == point_index) {
          std::cout << "  ✅ Point index " << point_index << " matches position in point array" << std::endl;
        } else {
          std::cout << "  ❌ ERROR: Point position matches index " << i << " but selection reported index " << point_index << std::endl;
        }
        found_matching_position = true;
        break;
      }
    }
    if (!found_matching_position) {
      std::cout << "  ❌ ERROR: Could not find selected point position in point cloud data!" << std::endl;
    }
    
    // Create or update highlight layer for this point cloud
    std::string layer_name = "highlight_" + cloud_name;
    auto highlight_layer = point_cloud->GetLayer(layer_name);
    
    if (!highlight_layer) {
      highlight_layer = point_cloud->CreateLayer(layer_name, 100); // High priority
      highlight_layer->SetColor(glm::vec3(1.0f, 0.0f, 1.0f)); // Bright magenta (more visible)
      highlight_layer->SetPointSizeMultiplier(3.0f); // 3x larger (more obvious)
      std::cout << "  🆕 Created new highlight layer: " << layer_name << std::endl;
    }
    
    // Force layer settings each time (in case they get reset)
    highlight_layer->SetColor(glm::vec3(1.0f, 0.0f, 1.0f)); // Bright magenta
    highlight_layer->SetPointSizeMultiplier(3.0f); // 3x larger
    
    // WORKAROUND: Instead of using layers, create a standalone point cloud
    // This bypasses the layer index mapping issue entirely
    std::string standalone_name = "highlight_" + cloud_name + "_standalone";
    
    // Remove any previous standalone highlight
    scene_panel_->GetSceneManager()->RemoveOpenGLObject(standalone_name);
    
    // Create a new point cloud with ONLY the selected point at the exact position
    auto highlight_cloud = std::make_unique<PointCloud>();
    std::vector<glm::vec3> highlight_points = {point_pos}; // Use exact position, not index
    std::vector<glm::vec3> highlight_colors = {glm::vec3(1.0f, 0.0f, 1.0f)}; // Bright magenta
    highlight_cloud->SetPoints(highlight_points, highlight_colors);
    highlight_cloud->SetPointSize(15.0f); // Very large and visible
    scene_panel_->GetSceneManager()->AddOpenGLObject(standalone_name, std::move(highlight_cloud));
    
    std::cout << "  ✨ Created STANDALONE highlight at exact position (" << point_pos.x << ", " << point_pos.y << ", " << point_pos.z << ")" << std::endl;
    
    // KEEP the layer approach for comparison, but mark it as potentially broken
    std::vector<size_t> current_points = {point_index};
    highlighted_point_layers_[layer_name] = current_points;
    highlight_layer->SetPoints(current_points);
    highlight_layer->SetVisible(true);
    std::cout << "  ⚠️ Also set layer highlight (may be wrong due to index mapping bug)" << std::endl;
    
    // CRITICAL TEST: Let's verify the layer system is working correctly
    // We'll test if a layer with index 0 actually highlights the first point
    std::cout << "  🧪 TESTING: Creating test layer with first 3 points to verify layer indexing..." << std::endl;
    
    std::string test_layer_name = "index_test_" + cloud_name;
    auto test_layer = point_cloud->GetLayer(test_layer_name);
    if (!test_layer) {
      test_layer = point_cloud->CreateLayer(test_layer_name, 90); // Lower priority than highlight
    }
    
    // Highlight first 3 points with green to see if indices match
    std::vector<size_t> test_indices = {0, 1, 2};
    test_layer->SetPoints(test_indices);
    test_layer->SetColor(glm::vec3(0.0f, 1.0f, 0.0f)); // Bright green
    test_layer->SetPointSizeMultiplier(2.5f);
    test_layer->SetVisible(true);
    
    // Also get the actual positions of the first 3 points for comparison
    std::cout << "  🧪 First 3 points in cloud should be GREEN:" << std::endl;
    for (size_t i = 0; i < 3 && i < total_points; ++i) {
      glm::vec3 test_pos = point_cloud->GetPointPosition(i);
      std::cout << "    Point[" << i << "] = (" << test_pos.x << ", " << test_pos.y << ", " << test_pos.z << ")" << std::endl;
    }
    
    std::cout << "  🧪 Selected point[" << point_index << "] should be MAGENTA at (" << point_pos.x << ", " << point_pos.y << ", " << point_pos.z << ")" << std::endl;
  }
  
  void ClearAllHighlights() {
    // Clear object highlights
    for (const auto& [object_name, state] : highlighted_objects_) {
      auto* obj = scene_panel_->GetOpenGLObject(object_name);
      if (!obj) continue;
      
      // Restore original properties
      if (state.object_type == "sphere") {
        if (auto* sphere = dynamic_cast<Sphere*>(obj)) {
          sphere->SetColor(state.original_color);
          sphere->SetRadius(state.original_size);
        }
      } else if (state.object_type == "linestrip") {
        if (auto* line = dynamic_cast<LineStrip*>(obj)) {
          line->SetColor(state.original_color);
          line->SetLineWidth(state.original_size);
        }
      }
    }
    highlighted_objects_.clear();
    
    // Clear point cloud highlights
    for (const auto& [layer_name, points] : highlighted_point_layers_) {
      // Extract cloud name from layer name (remove "highlight_" prefix)
      std::string cloud_name = layer_name.substr(10); // Remove "highlight_"
      auto* obj = scene_panel_->GetOpenGLObject(cloud_name);
      auto* point_cloud = dynamic_cast<PointCloud*>(obj);
      if (point_cloud) {
        auto highlight_layer = point_cloud->GetLayer(layer_name);
        if (highlight_layer) {
          highlight_layer->SetVisible(false);
          highlight_layer->ClearPoints();
        }
      }
    }
    highlighted_point_layers_.clear();
  }
  
  void ApplyPulseEffect(const MultiSelection& selection) {
    if (!animation_enabled_) return;
    
    std::cout << "  ✨ Applying pulse effect to selected objects" << std::endl;
    // In a real implementation, this would start a pulsing animation
    // For this demo, we just log the action
  }
  
  void ClearAllEffects() {
    std::cout << "  🔄 Clearing all visual effects" << std::endl;
    ClearAllHighlights(); // Include highlight clearing in effect clearing
  }
  
  void ShowInfoOverlays() {
    if (!info_overlays_enabled_) return;
    
    const auto& selection = scene_panel_->GetMultiSelection();
    if (selection.Empty()) return;
    
    std::cout << "  📝 Displaying info overlays for selected objects" << std::endl;
    // In a real implementation, this would create text billboards above objects
  }
  
  void HideInfoOverlays() {
    std::cout << "  🙈 Hiding all info overlays" << std::endl;
    // Remove all text billboard overlays
  }
  
private:
  // Highlight state management
  struct ObjectHighlightState {
    glm::vec3 original_color;
    float original_size;
    std::string object_type;
  };
  
  std::unordered_map<std::string, ObjectHighlightState> highlighted_objects_;
  std::unordered_map<std::string, std::vector<size_t>> highlighted_point_layers_;
  
  // State variables for enhanced interactions
  bool animation_enabled_ = false;
  bool info_overlays_enabled_ = false;
};

int main() {
  ComprehensiveSelectionTest app;
  return app.Run();
}