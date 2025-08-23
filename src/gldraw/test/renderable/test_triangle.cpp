/*
 * @file test_triangle.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Manual test for triangle rendering functionality in 2D mode
 *
 * This test creates a window displaying different triangle types in 2D for
 * UI elements, shapes, and geometric visualization.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/triangle.hpp"

using namespace quickviz;

void SetupTriangleScene(GlSceneManager* scene_manager) {
    // 1. Basic triangle - Orange
    auto basic_triangle = std::make_unique<Triangle>(1.0f);
    basic_triangle->SetColor(glm::vec3(1.0f, 0.5f, 0.2f));
    scene_manager->AddOpenGLObject("basic", std::move(basic_triangle));
    
    // 2. Small triangle - Red
    auto small_triangle = std::make_unique<Triangle>(0.5f);
    small_triangle->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
    scene_manager->AddOpenGLObject("small", std::move(small_triangle));
    
    // 3. Large triangle - Green
    auto large_triangle = std::make_unique<Triangle>(2.0f);
    large_triangle->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
    scene_manager->AddOpenGLObject("large", std::move(large_triangle));
    
    // 4. Blue triangle - Blue
    auto blue_triangle = std::make_unique<Triangle>(1.2f);
    blue_triangle->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));
    scene_manager->AddOpenGLObject("blue", std::move(blue_triangle));
    
    // 5. Yellow triangle - Yellow
    auto yellow_triangle = std::make_unique<Triangle>(0.8f);
    yellow_triangle->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));
    scene_manager->AddOpenGLObject("yellow", std::move(yellow_triangle));
    
    // 6. Purple triangle - Purple
    auto purple_triangle = std::make_unique<Triangle>(1.5f);
    purple_triangle->SetColor(glm::vec3(0.8f, 0.2f, 0.8f));
    scene_manager->AddOpenGLObject("purple", std::move(purple_triangle));
    
    // 7. Cyan triangle - Cyan
    auto cyan_triangle = std::make_unique<Triangle>(0.7f);
    cyan_triangle->SetColor(glm::vec3(0.0f, 1.0f, 1.0f));
    scene_manager->AddOpenGLObject("cyan", std::move(cyan_triangle));
    
    // 8. White triangle - White
    auto white_triangle = std::make_unique<Triangle>(1.3f);
    white_triangle->SetColor(glm::vec3(1.0f, 1.0f, 1.0f));
    scene_manager->AddOpenGLObject("white", std::move(white_triangle));
    
    // 9. Gray triangle - Gray
    auto gray_triangle = std::make_unique<Triangle>(0.9f);
    gray_triangle->SetColor(glm::vec3(0.5f, 0.5f, 0.5f));
    scene_manager->AddOpenGLObject("gray", std::move(gray_triangle));
    
    // 10. Very small triangle - Pink
    auto tiny_triangle = std::make_unique<Triangle>(0.3f);
    tiny_triangle->SetColor(glm::vec3(1.0f, 0.4f, 0.7f));
    scene_manager->AddOpenGLObject("tiny", std::move(tiny_triangle));
    
    // 11. Very large triangle - Dark Green
    auto huge_triangle = std::make_unique<Triangle>(2.5f);
    huge_triangle->SetColor(glm::vec3(0.0f, 0.5f, 0.0f));
    scene_manager->AddOpenGLObject("huge", std::move(huge_triangle));
    
    // 12. Golden triangle - Gold
    auto golden_triangle = std::make_unique<Triangle>(1.1f);
    golden_triangle->SetColor(glm::vec3(1.0f, 0.8f, 0.0f));
    scene_manager->AddOpenGLObject("golden", std::move(golden_triangle));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view for 2D mode
        GlView::Config config;
        config.window_title = "Triangle Rendering Test - 2D Mode";
        config.scene_mode = GlSceneManager::Mode::k2D;  // Set 2D mode
        config.show_grid = true;  // Disable grid for cleaner 2D view
        config.show_coordinate_frame = true;  // Disable 3D coordinate frame
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing triangle rendering in 2D mode for shapes and UI elements");
        
        view.AddHelpSection("Triangle Features Demonstrated", {
            "- Various triangle sizes (0.3f to 2.5f)",
            "- Different colors for visual distinction",
            "- 2D rendering mode for flat UI elements",
            "- Multiple triangles in single scene",
            "- Color variety: primary, secondary, and mixed colors"
        });
        
        view.AddHelpSection("2D Mode Features", {
            "- Orthographic projection for flat rendering",
            "- No 3D coordinate frame or grid",
            "- Optimized for UI and 2D graphics",
            "- All triangles render in same Z-plane",
            "- Camera controls adapted for 2D navigation"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- 12 triangles with different sizes and colors",
            "- Basic triangle (Orange, size 1.0)",
            "- Small/Large size variations (Red 0.5f, Green 2.0f)",
            "- Color spectrum demonstration",
            "- Extreme sizes: tiny (Pink 0.3f) and huge (Dark Green 2.5f)",
            "- Intermediate sizes with mixed colors"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupTriangleScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}