/*
 * @file test_sphere.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Test for Sphere rendering functionality
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/sphere.hpp"

using namespace quickviz;

void SetupSphereScene(GlSceneManager* scene_manager) {
    // 1. Basic sphere - Red
    auto sphere1 = std::make_unique<Sphere>(glm::vec3(0.0f, 0.0f, 0.0f), 1.0f);
    sphere1->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
    scene_manager->AddOpenGLObject("sphere1", std::move(sphere1));
    
    // 2. Large sphere - Green
    auto sphere2 = std::make_unique<Sphere>(glm::vec3(-4.0f, 0.0f, 0.0f), 2.0f);
    sphere2->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
    scene_manager->AddOpenGLObject("sphere2", std::move(sphere2));
    
    // 3. Small sphere - Blue
    auto sphere3 = std::make_unique<Sphere>(glm::vec3(3.0f, 0.0f, 0.0f), 0.5f);
    sphere3->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));
    scene_manager->AddOpenGLObject("sphere3", std::move(sphere3));
    
    // 4. Transparent sphere - Yellow
    auto sphere4 = std::make_unique<Sphere>(glm::vec3(0.0f, 3.0f, 0.0f), 1.5f);
    sphere4->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));
    sphere4->SetRenderMode(Sphere::RenderMode::kTransparent);
    sphere4->SetOpacity(0.6f);
    scene_manager->AddOpenGLObject("sphere4", std::move(sphere4));
    
    // 5. Wireframe sphere - Purple
    auto sphere5 = std::make_unique<Sphere>(glm::vec3(0.0f, -3.0f, 0.0f), 1.2f);
    sphere5->SetColor(glm::vec3(0.8f, 0.2f, 0.8f));
    sphere5->SetRenderMode(Sphere::RenderMode::kWireframe);
    scene_manager->AddOpenGLObject("sphere5", std::move(sphere5));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view for 3D mode
        GlView::Config config;
        config.window_title = "Sphere Rendering Test";
        config.coordinate_frame_size = 2.0f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing sphere rendering with various sizes, colors, and rendering modes");
        
        view.AddHelpSection("Sphere Features Demonstrated", {
            "- Different sphere radii (0.5f to 2.0f)",
            "- Various colors for visual distinction",
            "- Positioning in 3D space",
            "- Transparency effects",
            "- Wireframe rendering mode",
            "- Smooth spherical surfaces"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Red sphere: Basic 1.0 radius at origin",
            "- Green sphere: Large 2.0 radius at (-4,0,0)",
            "- Blue sphere: Small 0.5 radius at (3,0,0)",
            "- Yellow sphere: Transparent 1.5 radius at (0,3,0)",
            "- Purple sphere: Wireframe 1.2 radius at (0,-3,0)"
        });
        
        view.AddHelpSection("Applications", {
            "- 3D object representation",
            "- Particle system visualization",
            "- Bounding sphere display",
            "- Physics collision volumes",
            "- Robotics workspace boundaries",
            "- Mathematical 3D demonstrations"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupSphereScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}