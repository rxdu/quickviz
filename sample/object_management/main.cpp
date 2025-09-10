/*
 * bridge_test_app.cpp
 *
 * Created on: Sep 10, 2025
 * Description: Manual test application for SceneManagerBridge integration
 * 
 * This application provides an interactive UI to test and demonstrate
 * the SceneState ↔ GlSceneManager integration bridge functionality.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <vector>
#include <memory>

#include "imview/box.hpp"
#include "imview/viewer.hpp"
#include "imview/panel.hpp"

#include "gldraw/scene_manager.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/point_cloud.hpp"
#include "gldraw/renderable/sphere.hpp"
#include "gldraw/renderable/mesh.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"

#include "scenegraph/integration/scene_manager_bridge.hpp"

#include "bridge_scene_manager.hpp"
#include "bridge_control_panel.hpp"

using namespace quickviz;

// Generate test point cloud data
std::shared_ptr<PointCloud> CreateSpiralPointCloud(size_t num_points = 2000) {
    auto cloud = std::make_shared<PointCloud>();
    
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> colors;
    
    for (size_t i = 0; i < num_points; ++i) {
        float t = static_cast<float>(i) / num_points;
        float angle = t * 2.0f * M_PI * 8.0f;  // 8 spirals
        float radius = t * 3.0f;
        float height = t * 6.0f - 3.0f;
        
        points.emplace_back(
            radius * cos(angle),
            radius * sin(angle),
            height
        );
        
        // Color based on height
        float hue = (height + 3.0f) / 6.0f;  // Normalize to [0,1]
        colors.emplace_back(
            hue, 
            1.0f - hue, 
            0.5f + 0.5f * sin(angle)
        );
    }
    
    cloud->SetPoints(points, colors);
    cloud->SetPointSize(3.0f);
    return cloud;
}

// Generate test mesh data
std::shared_ptr<Mesh> CreateTestCube(glm::vec3 center = glm::vec3(0, 0, 0), float size = 1.0f) {
    auto mesh = std::make_shared<Mesh>();
    
    float half_size = size * 0.5f;
    std::vector<glm::vec3> vertices = {
        // Front face
        center + glm::vec3(-half_size, -half_size,  half_size),
        center + glm::vec3( half_size, -half_size,  half_size),
        center + glm::vec3( half_size,  half_size,  half_size),
        center + glm::vec3(-half_size,  half_size,  half_size),
        // Back face
        center + glm::vec3(-half_size, -half_size, -half_size),
        center + glm::vec3( half_size, -half_size, -half_size),
        center + glm::vec3( half_size,  half_size, -half_size),
        center + glm::vec3(-half_size,  half_size, -half_size)
    };
    
    std::vector<uint32_t> indices = {
        // Front face
        0, 1, 2,  2, 3, 0,
        // Back face
        4, 7, 6,  6, 5, 4,
        // Top face
        3, 2, 6,  6, 7, 3,
        // Bottom face
        0, 4, 5,  5, 1, 0,
        // Right face
        1, 5, 6,  6, 2, 1,
        // Left face
        0, 3, 7,  7, 4, 0
    };
    
    mesh->SetVertices(vertices);
    mesh->SetIndices(indices);
    
    return mesh;
}

int main(int argc, char* argv[]) {
    std::cout << "\n=== QuickViz SceneManagerBridge Test Application ===" << std::endl;
    std::cout << "Interactive test for SceneState ↔ GlSceneManager integration" << std::endl;
    std::cout << "Use the control panel to test different operation modes and features" << std::endl;

    try {
        // Create viewer for visualization
        std::cout << "\n=== Creating Visualization ===" << std::endl;
        Viewer viewer("SceneManagerBridge Test", 1400, 900);
        
        // Create main container box
        auto main_box = std::make_shared<Box>("main_container");
        main_box->SetFlexDirection(Styling::FlexDirection::kRow);
        main_box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
        main_box->SetAlignItems(Styling::AlignItems::kStretch);

        // Create the bridge scene manager (3D view)
        auto bridge_sm = std::make_shared<BridgeSceneManager>("Bridge Test Scene");
        bridge_sm->SetAutoLayout(true);
        bridge_sm->SetNoTitleBar(false);
        bridge_sm->SetFlexGrow(0.7f);   // Takes most of the space
        bridge_sm->SetFlexShrink(1.0f);

        // Create control panel (right side)
        auto control_panel = std::make_shared<BridgeControlPanel>("Bridge Controls", bridge_sm->GetBridge());
        control_panel->SetAutoLayout(true);
        control_panel->SetNoTitleBar(false);
        control_panel->SetFlexGrow(0.3f);   // Smaller control panel
        control_panel->SetFlexShrink(0.0f);

        // Connect the control panel to the scene manager
        bridge_sm->SetControlPanel(control_panel.get());

        // Pre-populate scene with some test objects
        std::cout << "Creating test objects..." << std::endl;
        
        // Add all initial objects through the bridge for consistent object management
        
        // Add coordinate frame for reference
        auto coord_frame = std::make_shared<CoordinateFrame>(2.0f);
        bridge_sm->GetBridge()->AddObject("coordinate_frame", coord_frame);
        
        // Add grid for reference
        auto grid = std::make_shared<Grid>(0.5f, 10.0f, glm::vec3(0.3f, 0.3f, 0.3f));
        bridge_sm->GetBridge()->AddObject("reference_grid", grid);
        
        // Add test point cloud
        auto spiral_cloud = CreateSpiralPointCloud(1500);
        bridge_sm->GetBridge()->AddObject("spiral_cloud", spiral_cloud);
        
        // Add test cubes at different positions
        auto cube1 = CreateTestCube(glm::vec3(-3, 0, 1), 0.8f);
        auto cube2 = CreateTestCube(glm::vec3(3, 0, 1), 1.2f);
        auto cube3 = CreateTestCube(glm::vec3(0, 3, 1), 1.0f);
        
        bridge_sm->GetBridge()->AddObject("cube_left", cube1);
        bridge_sm->GetBridge()->AddObject("cube_right", cube2);
        bridge_sm->GetBridge()->AddObject("cube_top", cube3);
        
        // Add some spheres
        auto sphere1 = std::make_shared<Sphere>(glm::vec3(-2, -2, 2), 0.6f);
        auto sphere2 = std::make_shared<Sphere>(glm::vec3(2, -2, 2), 0.4f);
        
        bridge_sm->GetBridge()->AddObject("sphere1", sphere1);
        bridge_sm->GetBridge()->AddObject("sphere2", sphere2);
        
        // Add test sphere for undo/redo functionality  
        auto test_sphere = std::make_shared<Sphere>(glm::vec3(0, 0, 3), 0.5f);
        bridge_sm->GetBridge()->AddObject("test_sphere_bridge", test_sphere);

        // Add components to main container
        main_box->AddChild(bridge_sm);
        main_box->AddChild(control_panel);

        // Add to viewer
        viewer.AddSceneObject(main_box);

        std::cout << "Visualization ready!" << std::endl;
        std::cout << "\nInstructions:" << std::endl;
        std::cout << "- Use the control panel on the right to test bridge features" << std::endl;
        std::cout << "- Switch operation modes (Direct/Immediate/Recorded)" << std::endl;
        std::cout << "- Try transformations and see undo/redo in action" << std::endl;
        std::cout << "- Add/remove objects dynamically" << std::endl;
        std::cout << "- Monitor statistics and memory usage" << std::endl;
        std::cout << "- Close the window to exit" << std::endl;
        
        viewer.Show();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}