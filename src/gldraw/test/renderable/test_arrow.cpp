/*
 * @file test_arrow.cpp
 * @author Ruixiang Du (ruixiang.du@gmail.com)
 * @date 2025-08-23
 * @brief Manual test for arrow rendering functionality
 *
 * This test creates a window displaying different arrow types for vectors and directions.
 * Run this test to visually verify arrow functionality for robotics applications.
 *
 * Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

#include "gldraw/gl_view.hpp"
#include "gldraw/renderable/arrow.hpp"

using namespace quickviz;

void SetupArrowScene(GlSceneManager* scene_manager) {
    // 1. X-axis arrow - Red
    auto x_arrow = std::make_unique<Arrow>(
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(2.0f, 0.0f, 0.0f)
    );
    x_arrow->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
    x_arrow->SetShaftRadius(0.03f);
    x_arrow->SetHeadRadius(0.06f);
    scene_manager->AddOpenGLObject("x_arrow", std::move(x_arrow));
    
    // 2. Y-axis arrow - Green
    auto y_arrow = std::make_unique<Arrow>();
    y_arrow->SetDirection(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), 2.0f);
    y_arrow->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
    y_arrow->SetShaftRadius(0.03f);
    y_arrow->SetHeadRadius(0.06f);
    scene_manager->AddOpenGLObject("y_arrow", std::move(y_arrow));
    
    // 3. Z-axis arrow - Blue
    auto z_arrow = std::make_unique<Arrow>(
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 0.0f, 2.0f)
    );
    z_arrow->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));
    z_arrow->SetShaftRadius(0.03f);
    z_arrow->SetHeadRadius(0.06f);
    scene_manager->AddOpenGLObject("z_arrow", std::move(z_arrow));
    
    // 4. Velocity vector - Yellow, thinner
    auto velocity_arrow = std::make_unique<Arrow>(
        glm::vec3(3.0f, 0.0f, 0.0f),
        glm::vec3(4.5f, 1.0f, 0.5f)
    );
    velocity_arrow->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));
    velocity_arrow->SetShaftRadius(0.02f);
    velocity_arrow->SetHeadRadius(0.04f);
    velocity_arrow->SetHeadLengthRatio(0.3f);
    scene_manager->AddOpenGLObject("velocity", std::move(velocity_arrow));
    
    // 5. Force vector - Purple, thick
    auto force_arrow = std::make_unique<Arrow>(
        glm::vec3(-3.0f, 0.0f, 0.0f),
        glm::vec3(-3.0f, 2.0f, 0.0f)
    );
    force_arrow->SetColor(glm::vec3(0.8f, 0.2f, 0.8f));
    force_arrow->SetShaftRadius(0.05f);
    force_arrow->SetHeadRadius(0.1f);
    force_arrow->SetHeadLengthRatio(0.15f);
    scene_manager->AddOpenGLObject("force", std::move(force_arrow));
    
    // 6. Diagonal arrow - Cyan
    auto diagonal_arrow = std::make_unique<Arrow>(
        glm::vec3(-2.0f, 0.0f, -2.0f),
        glm::vec3(2.0f, 1.5f, 2.0f)
    );
    diagonal_arrow->SetColor(glm::vec3(0.0f, 1.0f, 1.0f));
    scene_manager->AddOpenGLObject("diagonal", std::move(diagonal_arrow));
    
    // 7. Simple line arrow (for performance) - Orange
    auto line_arrow = std::make_unique<Arrow>(
        glm::vec3(0.0f, 0.0f, -3.0f),
        glm::vec3(2.0f, 0.0f, -3.0f)
    );
    line_arrow->SetColor(glm::vec3(1.0f, 0.5f, 0.0f));
    line_arrow->SetShowAsLine(true);
    scene_manager->AddOpenGLObject("line_arrow", std::move(line_arrow));
    
    // 8. Array of small arrows (e.g., vector field)
    for (int i = -2; i <= 2; ++i) {
        for (int j = -2; j <= 2; ++j) {
            if (i == 0 && j == 0) continue;  // Skip origin
            
            float x = i * 1.5f;
            float z = j * 1.5f + 5.0f;
            float angle = atan2(j, i);
            
            auto field_arrow = std::make_unique<Arrow>(
                glm::vec3(x, 0.0f, z),
                glm::vec3(x + 0.5f * cos(angle), 0.2f, z + 0.5f * sin(angle))
            );
            field_arrow->SetColor(glm::vec3(0.5f, 0.5f, 1.0f));
            field_arrow->SetShaftRadius(0.01f);
            field_arrow->SetHeadRadius(0.025f);
            field_arrow->SetResolution(8);  // Lower resolution for many arrows
            
            scene_manager->AddOpenGLObject(
                "field_" + std::to_string(i) + "_" + std::to_string(j),
                std::move(field_arrow)
            );
        }
    }
    
    // 9. High-resolution arrow - White
    auto hires_arrow = std::make_unique<Arrow>(
        glm::vec3(0.0f, 3.0f, 0.0f),
        glm::vec3(2.0f, 3.0f, 0.0f)
    );
    hires_arrow->SetColor(glm::vec3(1.0f, 1.0f, 1.0f));
    hires_arrow->SetShaftRadius(0.04f);
    hires_arrow->SetHeadRadius(0.08f);
    hires_arrow->SetResolution(32);  // High resolution for smooth appearance
    scene_manager->AddOpenGLObject("hires", std::move(hires_arrow));
}

int main(int argc, char* argv[]) {
    try {
        // Configure the view
        GlView::Config config;
        config.window_title = "Arrow Rendering Test";
        config.coordinate_frame_size = 1.0f;
        
        // Create the view
        GlView view(config);
        
        // Set up description and help sections
        view.SetDescription("Testing arrow rendering for vectors, directions, and forces");
        
        view.AddHelpSection("Arrow Features Demonstrated", {
            "- Basic directional arrows",
            "- Variable shaft and head sizes",
            "- Different colors for different purposes",
            "- Simple line mode for performance",
            "- Vector fields with many arrows",
            "- Variable resolution for quality/performance trade-off"
        });
        
        view.AddHelpSection("Scene Contents", {
            "- Reference grid and coordinate frame",
            "- X/Y/Z axis arrows (Red/Green/Blue)",
            "- Velocity vector (Yellow, thin)",
            "- Force vector (Purple, thick)",
            "- Diagonal 3D arrow (Cyan)",
            "- Simple line arrow (Orange)",
            "- Vector field (5x5 grid of small blue arrows)",
            "- High-resolution arrow (White)"
        });
        
        // Set the scene setup callback
        view.SetSceneSetup(SetupArrowScene);
        
        // Run the view
        view.Run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}