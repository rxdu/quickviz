/*
 * @file test_cylinder.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Test for Cylinder rendering functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>

#include "scene/scene_app.hpp"
#include "scene/renderable/cylinder.hpp"

using namespace quickviz;

void SetupCylinderScene(SceneManager* scene_manager) {
    // 1. Basic cylinder - Red
    auto cylinder1 = std::make_unique<Cylinder>(glm::vec3(0.0f, 0.0f, 0.0f), 2.0f, 1.0f);
    cylinder1->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
    scene_manager->AddOpenGLObject("cylinder1", std::move(cylinder1));
    
    // 2. Wide short cylinder - Green
    auto cylinder2 = std::make_unique<Cylinder>(glm::vec3(-3.0f, 0.0f, 0.0f), 1.0f, 1.5f);
    cylinder2->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
    scene_manager->AddOpenGLObject("cylinder2", std::move(cylinder2));
    
    // 3. Thin tall cylinder - Blue
    auto cylinder3 = std::make_unique<Cylinder>(glm::vec3(3.0f, 0.0f, 0.0f), 3.0f, 0.5f);
    cylinder3->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));
    scene_manager->AddOpenGLObject("cylinder3", std::move(cylinder3));
    
    // 4. Transparent cylinder - Yellow
    auto cylinder4 = std::make_unique<Cylinder>(glm::vec3(0.0f, 0.0f, 3.0f), 1.5f, 0.8f);
    cylinder4->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));
    cylinder4->SetRenderMode(Cylinder::RenderMode::kTransparent);
    cylinder4->SetOpacity(0.6f);
    scene_manager->AddOpenGLObject("cylinder4", std::move(cylinder4));
    
    // 5. Wireframe cylinder - White (for maximum contrast)
    auto cylinder5 = std::make_unique<Cylinder>(glm::vec3(0.0f, 0.0f, -3.0f), 1.8f, 1.2f);
    cylinder5->SetColor(glm::vec3(1.0f, 1.0f, 1.0f));  // White for maximum contrast
    cylinder5->SetWireframeColor(glm::vec3(1.0f, 1.0f, 1.0f));  // White wireframe
    cylinder5->SetRenderMode(Cylinder::RenderMode::kWireframe);
    cylinder5->SetWireframeWidth(2.0f);  // Make lines thicker for visibility
    scene_manager->AddOpenGLObject("cylinder5", std::move(cylinder5));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view for 3D mode
        SceneApp::Config config;
        config.window_title = "Cylinder Rendering Test";
        config.coordinate_frame_size = 2.0f;
        
        // Create the view
        SceneApp view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing cylinder rendering with various dimensions and rendering modes");
        
        view.AddHelpSection("Cylinder Features Demonstrated", {
            "- Different radius and height combinations",
            "- Various colors for visual distinction",
            "- Positioning in 3D space",
            "- Transparency effects",
            "- Wireframe rendering mode",
            "- Smooth cylindrical surfaces"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Red cylinder: Basic 1.0 radius, 2.0 height at origin",
            "- Green cylinder: Wide 1.5 radius, 1.0 height at (-3,0,0)",
            "- Blue cylinder: Thin 0.5 radius, 3.0 height at (3,0,0)",
            "- Yellow cylinder: Transparent at (0,0,3)",
            "- White cylinder: Wireframe at (0,0,-3)"
        });
        
        view.AddHelpSection("Applications", {
            "- Cylindrical obstacle representation",
            "- Robotics joint visualization",
            "- Structural column modeling",
            "- Physics collision volumes",
            "- Industrial component display",
            "- Geometric shape demonstrations"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupCylinderScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}