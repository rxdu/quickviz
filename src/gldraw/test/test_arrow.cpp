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

#include "imview/viewer.hpp"
#include "imview/box.hpp"
#include "gldraw/gl_scene_manager.hpp"
#include "gldraw/renderable/arrow.hpp"
#include "gldraw/renderable/grid.hpp"
#include "gldraw/renderable/coordinate_frame.hpp"

using namespace quickviz;

class ArrowTestDemo {
public:
    ArrowTestDemo() {
        SetupViewer();
    }
    
    void SetupViewer() {
        // Create box container for layout
        auto box = std::make_shared<Box>("main_box");
        box->SetFlexDirection(Styling::FlexDirection::kRow);
        box->SetJustifyContent(Styling::JustifyContent::kFlexStart);
        box->SetAlignItems(Styling::AlignItems::kStretch);
        
        // Create scene manager with proper layout settings
        scene_manager_ = std::make_shared<GlSceneManager>("Arrow Rendering Test");
        scene_manager_->SetAutoLayout(true);
        scene_manager_->SetNoTitleBar(true);
        scene_manager_->SetFlexGrow(1.0f);
        scene_manager_->SetFlexShrink(0.0f);
        
        box->AddChild(scene_manager_);
        viewer_.AddSceneObject(box);
    }
    
    void CreateTestArrows() {
        // Add grid for reference
        auto grid = std::make_unique<Grid>(20.0f, 1.0f, glm::vec3(0.5f, 0.5f, 0.5f));
        scene_manager_->AddOpenGLObject("grid", std::move(grid));
        
        // Add coordinate frame at origin
        auto frame = std::make_unique<CoordinateFrame>(1.0f);
        scene_manager_->AddOpenGLObject("frame", std::move(frame));
        
        // 1. X-axis arrow - Red
        auto x_arrow = std::make_unique<Arrow>(
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(2.0f, 0.0f, 0.0f)
        );
        x_arrow->SetColor(glm::vec3(1.0f, 0.0f, 0.0f));
        x_arrow->SetShaftRadius(0.03f);
        x_arrow->SetHeadRadius(0.06f);
        scene_manager_->AddOpenGLObject("x_arrow", std::move(x_arrow));
        
        // 2. Y-axis arrow - Green
        auto y_arrow = std::make_unique<Arrow>();
        y_arrow->SetDirection(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), 2.0f);
        y_arrow->SetColor(glm::vec3(0.0f, 1.0f, 0.0f));
        y_arrow->SetShaftRadius(0.03f);
        y_arrow->SetHeadRadius(0.06f);
        scene_manager_->AddOpenGLObject("y_arrow", std::move(y_arrow));
        
        // 3. Z-axis arrow - Blue
        auto z_arrow = std::make_unique<Arrow>(
            glm::vec3(0.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, 0.0f, 2.0f)
        );
        z_arrow->SetColor(glm::vec3(0.0f, 0.0f, 1.0f));
        z_arrow->SetShaftRadius(0.03f);
        z_arrow->SetHeadRadius(0.06f);
        scene_manager_->AddOpenGLObject("z_arrow", std::move(z_arrow));
        
        // 4. Velocity vector - Yellow, thinner
        auto velocity_arrow = std::make_unique<Arrow>(
            glm::vec3(3.0f, 0.0f, 0.0f),
            glm::vec3(4.5f, 1.0f, 0.5f)
        );
        velocity_arrow->SetColor(glm::vec3(1.0f, 1.0f, 0.0f));
        velocity_arrow->SetShaftRadius(0.02f);
        velocity_arrow->SetHeadRadius(0.04f);
        velocity_arrow->SetHeadLengthRatio(0.3f);
        scene_manager_->AddOpenGLObject("velocity", std::move(velocity_arrow));
        
        // 5. Force vector - Purple, thick
        auto force_arrow = std::make_unique<Arrow>(
            glm::vec3(-3.0f, 0.0f, 0.0f),
            glm::vec3(-3.0f, 2.0f, 0.0f)
        );
        force_arrow->SetColor(glm::vec3(0.8f, 0.2f, 0.8f));
        force_arrow->SetShaftRadius(0.05f);
        force_arrow->SetHeadRadius(0.1f);
        force_arrow->SetHeadLengthRatio(0.15f);
        scene_manager_->AddOpenGLObject("force", std::move(force_arrow));
        
        // 6. Diagonal arrow - Cyan
        auto diagonal_arrow = std::make_unique<Arrow>(
            glm::vec3(-2.0f, 0.0f, -2.0f),
            glm::vec3(2.0f, 1.5f, 2.0f)
        );
        diagonal_arrow->SetColor(glm::vec3(0.0f, 1.0f, 1.0f));
        scene_manager_->AddOpenGLObject("diagonal", std::move(diagonal_arrow));
        
        // 7. Simple line arrow (for performance) - Orange
        auto line_arrow = std::make_unique<Arrow>(
            glm::vec3(0.0f, 0.0f, -3.0f),
            glm::vec3(2.0f, 0.0f, -3.0f)
        );
        line_arrow->SetColor(glm::vec3(1.0f, 0.5f, 0.0f));
        line_arrow->SetShowAsLine(true);
        scene_manager_->AddOpenGLObject("line_arrow", std::move(line_arrow));
        
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
                
                scene_manager_->AddOpenGLObject(
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
        scene_manager_->AddOpenGLObject("hires", std::move(hires_arrow));
        
        std::cout << "\nCreated test scene with:" << std::endl;
        std::cout << "  - Reference grid and coordinate frame" << std::endl;
        std::cout << "  - X/Y/Z axis arrows (Red/Green/Blue)" << std::endl;
        std::cout << "  - Velocity vector (Yellow, thin)" << std::endl;
        std::cout << "  - Force vector (Purple, thick)" << std::endl;
        std::cout << "  - Diagonal 3D arrow (Cyan)" << std::endl;
        std::cout << "  - Simple line arrow (Orange)" << std::endl;
        std::cout << "  - Vector field (5x5 grid of small blue arrows)" << std::endl;
        std::cout << "  - High-resolution arrow (White)" << std::endl;
    }
    
    void Run() {
        CreateTestArrows();
        
        std::cout << "\n=== Camera Controls ===" << std::endl;
        std::cout << "Left Mouse: Rotate camera (orbit mode)" << std::endl;
        std::cout << "Middle Mouse: Translate/Pan in 3D space" << std::endl;
        std::cout << "Scroll Wheel: Zoom in/out" << std::endl;
        std::cout << "R: Reset camera to default position" << std::endl;
        std::cout << "ESC: Exit application" << std::endl;
        
        std::cout << "\n=== Arrow Features Demonstrated ===" << std::endl;
        std::cout << "- Basic directional arrows" << std::endl;
        std::cout << "- Variable shaft and head sizes" << std::endl;
        std::cout << "- Different colors for different purposes" << std::endl;
        std::cout << "- Simple line mode for performance" << std::endl;
        std::cout << "- Vector fields with many arrows" << std::endl;
        std::cout << "- Variable resolution for quality/performance trade-off" << std::endl;
        
        // Show viewer
        viewer_.Show();
    }
    
private:
    Viewer viewer_;
    std::shared_ptr<GlSceneManager> scene_manager_;
};

int main(int argc, char* argv[]) {
    std::cout << "=== Arrow Rendering Test ===" << std::endl;
    std::cout << "Testing arrow rendering for vectors, directions, and forces\n" << std::endl;
    
    try {
        ArrowTestDemo demo;
        demo.Run();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}